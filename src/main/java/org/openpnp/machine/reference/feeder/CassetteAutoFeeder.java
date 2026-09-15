package org.openpnp.machine.reference.feeder;

import java.util.List;
import java.util.Map;

import org.openpnp.ConfigurationListener;
import org.openpnp.gui.MainFrame;
import org.openpnp.gui.support.PropertySheetWizardAdapter;
import org.openpnp.gui.support.Wizard;
import org.openpnp.machine.reference.feeder.wizards.CassetteAutoFeederConfigurationWizard;
import org.openpnp.machine.reference.feeder.wizards.ReferenceStripFeederConfigurationWizard;
import org.openpnp.model.Configuration;
import org.openpnp.model.Length;
import org.openpnp.model.LengthUnit;
import org.openpnp.model.Location;
import org.openpnp.model.Part;
import org.openpnp.model.Point;
import org.openpnp.spi.Actuator;
import org.openpnp.spi.Camera;
import org.openpnp.spi.Feeder;
import org.openpnp.spi.Machine;
import org.openpnp.spi.Nozzle;
import org.openpnp.spi.PropertySheetHolder;
import org.openpnp.util.MovableUtils;
import org.openpnp.util.OpenCvUtils;
import org.openpnp.util.Utils2D;
import org.openpnp.util.VisionUtils;
import org.openpnp.vision.pipeline.CvPipeline;
import org.openpnp.vision.pipeline.CvStage;
import org.simpleframework.xml.Element;
import org.pmw.tinylog.Logger;

/**
 * CassetteAutoFeeder — firmware-driven auto-advancing tape feeder. The physical tape
 * advance is delegated to the 3DPlacer feeder firmware via M888 g-code, while vision-based
 * sprocket-hole detection (inherited from {@link ReferenceStripFeeder}) provides precise
 * part positioning.
 *
 * <p>Unlike {@link CassetteFeeder} which uses push-pull mechanical feed, this feeder
 * simply sends an M888 command per hole advance and lets the firmware handle the motor.</p>
 *
 * <p>The shared baseplate layout (origin, spacing, orientation, totals, …) is owned by a
 * separate {@link CassetteFeederConfigurator} feeder, which is the single source of truth
 * and the trigger for discovery. This feeder holds only its per-slot state (row, col,
 * contact offsets, height, locations, feed counters) and reads baseplate geometry from its
 * configurator at runtime. At least one {@link CassetteFeederConfigurator} must therefore
 * be kept in the machine.</p>
 *
 * <h3>Multi-pitch</h3>
 * When part pitch &lt; sprocket-hole pitch (e.g. 2mm parts on 4mm holes), multiple parts
 * fit between consecutive holes. {@code subFeedIndex} cycles 0→(partsPerHole-1) and
 * {@link #getPickLocation()} offsets along the tape axis accordingly.
 * M888 is only sent when {@code subFeedIndex == 0} (new hole).
 */
public class CassetteAutoFeeder extends ReferenceStripFeeder {

    // ---- per-slot state ----
    @Element(required = false)
    protected int row = 0;
    @Element(required = false)
    protected int col = 0;
    @Element(required = false)
    protected int subType = 0;

    // Per-feeder contact offsets from firmware flash
    @Element(required = false)
    protected double feederContactOffsetX = 0;
    @Element(required = false)
    protected double feederContactOffsetY = 0;
    @Element(required = false)
    protected Length feederHeight = new Length(0, LengthUnit.Millimeters);

    // ---- multi-pitch ----
    @Element(required = false)
    protected int subFeedIndex = 0;

    /*
     * Legacy baseplate-layout fields. These USED to be the live configuration but
     * have moved to {@link CassetteFeederConfigurator} (the single source of truth).
     * They are kept here ONLY so old machine.xml files still load — OpenPnP's
     * SimpleXML serializer is strict and rejects unmapped elements. They are NOT
     * read at runtime (everything delegates to the configurator via
     * {@link #findConfigurator()}). On the first load after this refactor,
     * {@link #migrateLegacyBaseplate()} seeds a new CassetteFeederConfigurator
     * from them, so no existing layout is lost. They re-serialize harmlessly.
     */
    @Element(required = false)
    protected Length baseplateOffsetX = new Length(0, LengthUnit.Millimeters);
    @Element(required = false)
    protected Length baseplateOffsetY = new Length(0, LengthUnit.Millimeters);
    @Element(required = false)
    protected Length baseplateOffsetZ = new Length(0, LengthUnit.Millimeters);
    @Element(required = false)
    protected Length rowSpacing = new Length(8, LengthUnit.Millimeters);
    @Element(required = false)
    protected double rowDirectionX = 1.0;
    @Element(required = false)
    protected double rowDirectionY = 0.0;
    @Element(required = false)
    protected int continuousRowCol = 0;
    @Element(required = false)
    protected int skippedRowCol = 0;
    @Element(required = false)
    protected boolean isVerticalLayout = true;
    @Element(required = false)
    protected int orientation = 0;
    @Element(required = false)
    protected int totalRow = 30;
    @Element(required = false)
    protected int totalCol = 15;

    public CassetteAutoFeeder() {
        super();
        Configuration.get().addListener(new ConfigurationListener.Adapter() {
            @Override
            public void configurationLoaded(Configuration configuration) throws Exception {
                migrateLegacyBaseplate();
            }
        });
        setVisionEnabled(true);
    }

    @Override
    public Wizard getConfigurationWizard() {
        return new CassetteAutoFeederConfigurationWizard(this);
    }

    @Override
    public PropertySheetHolder.PropertySheet[] getPropertySheets() {
        return new PropertySheetHolder.PropertySheet[] {
                new PropertySheetWizardAdapter(
                        new ReferenceStripFeederConfigurationWizard(this),
                        "Strip Feeder"),
                new PropertySheetWizardAdapter(
                        new CassetteAutoFeederConfigurationWizard(this),
                        "Cassette Setup"),
        };
    }

    // ---- configurator lookup ----

    /**
     * Find this feeder's {@link CassetteFeederConfigurator} (the single source of
     * truth for baseplate layout). Throws if none exists — the operator must keep
     * at least one configurator in the machine.
     */
    public CassetteFeederConfigurator findConfigurator() {
        for (Feeder f : Configuration.get().getMachine().getFeeders()) {
            if (f instanceof CassetteFeederConfigurator) {
                return (CassetteFeederConfigurator) f;
            }
        }
        throw new IllegalStateException("No CassetteFeederConfigurator found in the machine. "
                + "Add one and configure the baseplate layout before using CassetteAutoFeeders.");
    }

    /**
     * One-time migration: if no {@link CassetteFeederConfigurator} exists yet,
     * create one seeded from the legacy baseplate-layout fields of the feeder
     * that has real (non-default) data. Runs on configurationLoaded; the first
     * CassetteAutoFeeder to run it creates the configurator, the rest skip it.
     * This preserves the baseplate layout when upgrading to the configurator
     * model without forcing the user to re-enter it.
     */
    private void migrateLegacyBaseplate() {
        Machine machine = Configuration.get().getMachine();
        for (Feeder f : machine.getFeeders()) {
            if (f instanceof CassetteFeederConfigurator) {
                return; // a configurator already exists (user-added or migrated)
            }
        }
        // Pick a donor feeder with real legacy data (origin offset != 0).
        CassetteAutoFeeder donor = null;
        for (Feeder f : machine.getFeeders()) {
            if (f instanceof CassetteAutoFeeder) {
                CassetteAutoFeeder cf = (CassetteAutoFeeder) f;
                if (cf.baseplateOffsetX != null
                        && cf.baseplateOffsetX.getValue() != 0) {
                    donor = cf;
                    break;
                }
            }
        }
        if (donor == null) {
            donor = this; // fall back to this feeder's (possibly default) fields
        }
        try {
            CassetteFeederConfigurator cfg = new CassetteFeederConfigurator();
            cfg.setName("CassetteFeederConfigurator");
            // The configurator is not pickable, so it has no Part. AbstractFeeder.partId
            // is a *required* @Attribute that defaults to null — without an explicit
            // empty part here, configuration.save() throws AttributeException and the
            // shutdown dialog blocks forever (machine.xml is never written).
            cfg.setPart(null);
            cfg.setBaseplateOffsetX(donor.baseplateOffsetX);
            cfg.setBaseplateOffsetY(donor.baseplateOffsetY);
            cfg.setBaseplateOffsetZ(donor.baseplateOffsetZ);
            cfg.setRowSpacing(donor.rowSpacing);
            cfg.setRowDirectionX(donor.rowDirectionX);
            cfg.setRowDirectionY(donor.rowDirectionY);
            cfg.setContinuousRowCol(donor.continuousRowCol);
            cfg.setSkippedRowCol(donor.skippedRowCol);
            cfg.setIsVerticalLayout(donor.isVerticalLayout);
            cfg.setOrientation(donor.orientation);
            cfg.setTotalRow(donor.totalRow);
            cfg.setTotalCol(donor.totalCol);
            machine.addFeeder(cfg);
            Logger.info("Migrated baseplate layout from {} into a new CassetteFeederConfigurator.",
                    donor.getName());
        }
        catch (Exception e) {
            Logger.error(e, "Failed to migrate legacy baseplate layout");
        }
    }

    // ---- accessors ----
    public int getRow() {
        return row;
    }
    public void setRow(int v) {
        Object o = row;
        row = v;
        firePropertyChange("row", o, v);
    }
    public int getCol() {
        return col;
    }
    public void setCol(int v) {
        Object o = col;
        col = v;
        firePropertyChange("col", o, v);
    }
    public int getSubType() {
        return subType;
    }
    public void setSubType(int v) {
        Object o = subType;
        subType = v;
        firePropertyChange("subType", o, v);
    }
    public double getFeederContactOffsetX() {
        return feederContactOffsetX;
    }
    public void setFeederContactOffsetX(double v) {
        Object o = feederContactOffsetX;
        feederContactOffsetX = v;
        firePropertyChange("feederContactOffsetX", o, v);
    }
    public double getFeederContactOffsetY() {
        return feederContactOffsetY;
    }
    public void setFeederContactOffsetY(double v) {
        Object o = feederContactOffsetY;
        feederContactOffsetY = v;
        firePropertyChange("feederContactOffsetY", o, v);
    }
    public Length getFeederHeight() {
        return feederHeight;
    }
    public void setFeederHeight(Length v) {
        Object o = feederHeight;
        feederHeight = v;
        firePropertyChange("feederHeight", o, v);
    }
    public int getSubFeedIndex() {
        return subFeedIndex;
    }
    public void setSubFeedIndex(int v) {
        Object o = subFeedIndex;
        subFeedIndex = v % Math.max(1, getPartsPerHole());
        firePropertyChange("subFeedIndex", o, subFeedIndex);
    }

    /** Computed: ceil(holePitch / partPitch). */
    public int getPartsPerHole() {
        double holeMm = getHolePitch().convertToUnits(LengthUnit.Millimeters).getValue();
        double partMm = getPartPitch().convertToUnits(LengthUnit.Millimeters).getValue();
        if (partMm <= 0) {
            return 1;
        }
        return (int) Math.ceil(holeMm / partMm);
    }

    // ---- discovery (driven by CassetteFeederConfigurator) ----

    /**
     * Apply firmware-reported per-slot info to this feeder, deriving its pick and
     * sprocket-hole locations from the configurator's baseplate layout. Called by
     * {@link CassetteFeederConfigurator#discoverFeeders()} for each reported slot.
     */
    public void applyDiscovery(CassetteFeederConfigurator cfg, Map<String, String> info) {
        int r = Integer.parseInt(info.get("r"));
        int c = Integer.parseInt(info.get("c"));
        double ox = info.containsKey("ox") ? Double.parseDouble(info.get("ox")) : 0;
        double oy = info.containsKey("oy") ? Double.parseDouble(info.get("oy")) : 0;
        double h = Double.parseDouble(info.get("h")) / 10.0;
        double pi = info.containsKey("pi")
                ? Double.parseDouble(info.get("pi"))
                : getPartPitch().getValue() * 10;
        int st = info.containsKey("st") ? Integer.parseInt(info.get("st")) : 0;

        setRow(r);
        setCol(c);
        setSubType(st);
        setFeederContactOffsetX(ox);
        setFeederContactOffsetY(oy);
        setFeederHeight(new Length(h, LengthUnit.Millimeters));
        setPartPitch(new Length(pi / 10.0, LengthUnit.Millimeters));
        setName(info.get("n"));

        // Derive pick + sprocket-hole locations from the configurator's baseplate geometry.
        int orientation = cfg.getOrientation();
        Location slot = cfg.getSlotCenterMm(r, c);
        double sx = slot.getX();
        double sy = slot.getY();
        double tapeW = getTapeWidth().convertToUnits(LengthUnit.Millimeters).getValue();
        double fDim = CassetteFeederConfigurator.tapeWidthToF.getOrDefault(tapeW, 3.5);

        // Pick = slot center + rotate(contact offset, orientation).
        setLocation(new Location(LengthUnit.Millimeters,
                sx + CassetteFeederConfigurator.rotateX(ox, oy, orientation),
                sy + CassetteFeederConfigurator.rotateY(ox, oy, orientation),
                h, 0));

        // Reference hole (hole 1): the sprocket hole sits F to the side of the
        // part and 2mm along the tape (a vision search hint). See CassetteFeeder
        // docs / index.md for the full geometry rationale.
        double holeOffX = ox - fDim;
        double holeOffY = oy + CassetteFeederConfigurator.sprocketHoleLateralOffsetMm;
        double h1x = sx + CassetteFeederConfigurator.rotateX(holeOffX, holeOffY, orientation);
        double h1y = sy + CassetteFeederConfigurator.rotateY(holeOffX, holeOffY, orientation);
        setReferenceHoleLocation(new Location(LengthUnit.Millimeters, h1x, h1y, h, 0));

        // Last hole = one sprocket-hole pitch along the tape feed direction from
        // hole 1. The feed direction is the base vector rotated by the orientation.
        double baseFeedX = cfg.getIsVerticalLayout() ? 0 : -CassetteFeederConfigurator.sprocketHolePitchMm;
        double baseFeedY = cfg.getIsVerticalLayout() ? -CassetteFeederConfigurator.sprocketHolePitchMm : 0;
        double tapeDx = CassetteFeederConfigurator.rotateX(baseFeedX, baseFeedY, orientation);
        double tapeDy = CassetteFeederConfigurator.rotateY(baseFeedX, baseFeedY, orientation);
        setLastHoleLocation(new Location(LengthUnit.Millimeters,
                h1x + tapeDx, h1y + tapeDy, h, 0));

        Part part = CassetteFeederConfigurator.resolvePart(info.get("n"));
        if (part != null) {
            setPart(part);
        }
        setEnabled(true);
    }

    // ---- manual setup ----

    /**
     * Recalculate the per-feeder contact offsets (OX, OY) and feeder height from
     * the currently-captured {@link #getReferenceHoleLocation() reference hole},
     * then derive the matching Pick and Last-hole locations.
     *
     * <p>This is the manual-setup counterpart to {@link #applyDiscovery()}: the
     * firmware-reported OX/OY/H are only authoritative after “Save To Feeder”
     * has flashed them. On a fresh baseplate (or whenever the camera-captured Z
     * is more trustworthy than the firmware value) the operator jogs to the
     * reference sprocket hole, captures it, and invokes this method to populate
     * everything else. Orientation and Tape Width come from the configurator and
     * this feeder respectively.</p>
     *
     * <p>Geometry (inverse of {@code applyDiscovery}): the reference hole sits at
     * {@code slot + rotate(ox - F, oy + 2, orientation)} where F is the EIA-481
     * dimension for the tape width. Inverting that rotation yields the contact
     * offsets, from which the pick location ({@code slot + rotate(ox, oy, ori)})
     * and the last hole (one pitch further along the tape feed direction) follow
     * directly. Z is taken from the captured reference hole.</p>
     */
    public void calculateOffsetsFromRefHole() {
        CassetteFeederConfigurator cfg = findConfigurator();
        int orientation = cfg.getOrientation();

        Location refHole = getReferenceHoleLocation().convertToUnits(LengthUnit.Millimeters);
        Location slot = cfg.getSlotCenterMm(row, col);
        double sx = slot.getX();
        double sy = slot.getY();

        double tapeW = getTapeWidth().convertToUnits(LengthUnit.Millimeters).getValue();
        double fDim = CassetteFeederConfigurator.tapeWidthToF.getOrDefault(tapeW, 3.5);

        // Vector from slot center to the captured reference hole (machine frame).
        double a = refHole.getX() - sx;
        double b = refHole.getY() - sy;

        // Unrotate back to the feeder-local frame to recover (ox - F, oy + 2).
        double dx = CassetteFeederConfigurator.inverseRotateX(a, b, orientation);
        double dy = CassetteFeederConfigurator.inverseRotateY(a, b, orientation);
        double ox = dx + fDim;
        double oy = dy - CassetteFeederConfigurator.sprocketHoleLateralOffsetMm;
        double h = refHole.getZ();

        Logger.info("{} calculateOffsetsFromRefHole: ref=({:.3},{:.3},{:.3}) slot=({:.3},{:.3}) "
                + "orientation={} tapeW={:.1} F={:.2} -> ox={:.3}, oy={:.3}, h={:.3}",
                getName(), refHole.getX(), refHole.getY(), refHole.getZ(),
                sx, sy, orientation, tapeW, fDim, ox, oy, h);

        // Persist the derived contact offsets + height (these are what
        // “Save To Feeder” flashes to the firmware).
        setFeederContactOffsetX(ox);
        setFeederContactOffsetY(oy);
        setFeederHeight(new Length(h, LengthUnit.Millimeters));

        // Pick = slot center + rotate(contact offset, orientation).
        double pickX = sx + CassetteFeederConfigurator.rotateX(ox, oy, orientation);
        double pickY = sy + CassetteFeederConfigurator.rotateY(ox, oy, orientation);
        setLocation(new Location(LengthUnit.Millimeters, pickX, pickY, h, 0));

        // Last hole = one sprocket-hole pitch along the tape feed direction
        // from the (just-captured) reference hole.
        double baseFeedX = cfg.getIsVerticalLayout() ? 0 : -CassetteFeederConfigurator.sprocketHolePitchMm;
        double baseFeedY = cfg.getIsVerticalLayout() ? -CassetteFeederConfigurator.sprocketHolePitchMm : 0;
        double tapeDx = CassetteFeederConfigurator.rotateX(baseFeedX, baseFeedY, orientation);
        double tapeDy = CassetteFeederConfigurator.rotateY(baseFeedX, baseFeedY, orientation);
        setLastHoleLocation(new Location(LengthUnit.Millimeters,
                refHole.getX() + tapeDx, refHole.getY() + tapeDy, h, 0));

        Logger.info("{} calculateOffsetsFromRefHole: pick=({:.3},{:.3},{:.3}) lastHole=({:.3},{:.3},{:.3})",
                getName(), pickX, pickY, h,
                refHole.getX() + tapeDx, refHole.getY() + tapeDy, h);
    }

    // ---- feed ----

    /**
     * Auto-feeder feed semantics.
     * <p>Unlike the strip feeder (stationary tape, nozzle indexes along it), the
     * firmware physically advances the tape by one hole pitch (4mm) per M888.
     * After each advance the next sprocket hole arrives at the fixed pick
     * station, so the 1st component of the new hole group is always picked at
     * the same (vision-corrected) place.</p>
     *
     * <p>For multi-pitch tapes (part pitch &lt; hole pitch, e.g. 2mm parts on 4mm
     * holes) there are {@link #getPartsPerHole()} parts per hole group. Only the
     * first part of a group triggers a tape advance (M888); subsequent feeds in
     * the same group just move the nozzle to the next component without
     * advancing the tape.</p>
     *
     * <p>Example (partPitch=2mm, holePitch=4mm → ppHole=2):</p>
     * <ul>
     *   <li>feed 1 (sub=0): advance tape, vision, pick 1st component</li>
     *   <li>feed 2 (sub=1): no advance, pick 2nd component</li>
     *   <li>feed 3 (sub=0): advance tape, vision, pick 1st component</li>
     * </ul>
     */
    @Override
    public void feed(Nozzle nozzle) throws Exception {
        if (getFeedOptions() != FeedOptions.Normal) {
            if (getFeedOptions() == FeedOptions.SkipNext) {
                setFeedOptions(FeedOptions.Normal);
            }
            return;
        }

        int ppHole = Math.max(1, getPartsPerHole());

        // subFeedIndex is the component-within-hole-group we are about to feed.
        // The firmware advances by one HOLE pitch (4mm) per M888 command; for
        // multi-pitch tapes we only send M888 when starting a new hole group.
        if (subFeedIndex == 0) {
            Actuator act = CassetteFeederConfigurator.configureActuator();
            CassetteFeederConfigurator cfg = findConfigurator();
            String cmd = String.format(
                    "R%d C%d TR%d TC%d AD1;",
                    row, col, cfg.getTotalRow(), cfg.getTotalCol());
            String resp = act.read(cmd);
            Logger.info("{} feed (advance to next hole, sub {}/{}): {}",
                    getName(), subFeedIndex + 1, ppHole, resp);

            if (isVisionEnabled()) {
                updateHoleVision(nozzle);
            }
        }
        else {
            Logger.info("{} feed (sub {}/{}, no tape advance)", getName(),
                    subFeedIndex + 1, ppHole);
        }

        // Advance the sub index for the next feed. The pick location is derived
        // from feedCount (see getPickLocation) so it stays consistent regardless
        // of the feed/pick call order.
        subFeedIndex = (subFeedIndex + 1) % ppHole;
        setFeedCount(getFeedCount() + 1);
    }

    /**
     * Locate the sprocket hole that is now at the pick station after a tape
     * advance.
     * <p>Because the firmware advances the tape by exactly one hole pitch, the
     * next hole arrives at (approximately) the <b>same fixed machine position</b>
     * as the previous one — not one hole-pitch further along. Vision is used to
     * correct the small per-advance positioning error (backlash, slip, ...).</p>
     * <p>Unlike a stationary strip feeder, a failure to find the hole is fatal
     * here: the tape has already physically advanced, so we cannot fall back to
     * a calculated position. We re-throw so the operator sees the problem and
     * the feed stops rather than picking from an uncorrected location.</p>
     */
    private void updateHoleVision(Nozzle nozzle) throws Exception {
        Camera camera = nozzle.getHead().getDefaultCamera();
        ensureFeederZ(camera);

        Location expectedHole;
        if (visionLocation == null) {
            // Very first feed: look at the configured reference hole.
            expectedHole = getReferenceHoleLocation();
        }
        else {
            // Subsequent feeds: the new hole is where the last one was measured.
            expectedHole = visionLocation;
        }

        Location found = findClosestHole(camera, expectedHole);

        // Sanity check: reject a detection that is implausibly far from where we
        // expect the hole (mirrors the parent strip feeder's 2mm guard).
        double distanceMm = found.getLinearLengthTo(expectedHole)
                .convertToUnits(LengthUnit.Millimeters).getValue();
        if (distanceMm > 2.0) {
            throw new FeederEmptyException(String.format(
                    "Feeder %s: detected sprocket hole is %.2fmm away from the "
                            + "expected position — tape not advanced correctly?",
                    getName(), distanceMm));
        }

        if (visionLocationReference == null) {
            // Record the first measured hole; it anchors the tape axis together
            // with the configured last hole.
            visionLocationReference = found;
        }
        visionLocation = found;
        Logger.info("{} vision: hole at {} (Δ{:.2}mm from expected)",
                getName(), found, distanceMm);
    }

    /**
     * Wide-search sprocket-hole detection.
     * <p>The parent strip feeder restricts the search to a 2mm radius around the
     * expected hole (plus an image mask), which is fine when the reference hole
     * was hand-placed in the wizard. For this auto-feeder the reference hole is
     * derived from the baseplate geometry model and is only approximate, so a
     * 2mm radius reliably finds nothing. Instead we search the whole camera view
     * (like the strip-feeder wizard does) and return the detected hole closest to
     * the expected location.</p>
     */
    @Override
    protected Location findClosestHole(Camera camera, Location expectedLocation) throws Exception {
        MovableUtils.moveToLocationAtSafeZ(camera, expectedLocation);

        try (CvPipeline pipeline = getPipeline()) {
            Integer pxMinDistance = (int) VisionUtils.toPixels(getHolePitchMin(), camera);
            Integer pxMinDiameter = (int) VisionUtils.toPixels(getHoleDiameterMin(), camera);
            Integer pxMaxDiameter = (int) VisionUtils.toPixels(getHoleDiameterMax(), camera);

            pipeline.setProperty("camera", camera);
            pipeline.setProperty("feeder", this);
            pipeline.setProperty("DetectFixedCirclesHough.minDistance", pxMinDistance);
            pipeline.setProperty("DetectFixedCirclesHough.minDiameter", pxMinDiameter);
            pipeline.setProperty("DetectFixedCirclesHough.maxDiameter", pxMaxDiameter);
            pipeline.setProperty("sprocketHole.diameter", getHoleDiameter());

            // Search the entire camera field of view (mirrors the strip feeder
            // configuration wizard), not the parent's 2mm radius, since the
            // expected location is only approximately known.
            Length range = camera.getWidth() > camera.getHeight()
                    ? camera.getUnitsPerPixelAtZ().getLengthY().multiply(camera.getHeight() / 2.0)
                    : camera.getUnitsPerPixelAtZ().getLengthX().multiply(camera.getWidth() / 2.0);
            pipeline.setProperty("sprocketHole.maxDistance", range);
            // Nominal center = where the camera is looking (= expected location).
            pipeline.setProperty("sprocketHole.center", expectedLocation);
            // No MaskCircle: the default pipeline has no MaskCircle stage and we
            // want the full view searched.

            pipeline.process();

            // Show the detection result on the camera view, like the parent does.
            if (MainFrame.get() != null) {
                try {
                    MainFrame.get().getCameraViews().getCameraView(camera)
                            .showFilteredImage(OpenCvUtils.toBufferedImage(pipeline.getWorkingImage()), 250);
                }
                catch (Exception e) {
                    // not running in the UI
                }
            }

            List<CvStage.Result.Circle> results = pipeline.getExpectedResult(VisionUtils.PIPELINE_RESULTS_NAME)
                    .getExpectedListModel(CvStage.Result.Circle.class,
                            new FeederEmptyException("Feeder " + getName() + ": No tape holes found."));

            // Return the detected hole closest to the expected location.
            results.sort((a, b) -> {
                Double da = VisionUtils.getPixelLocation(camera, a.x, a.y)
                        .getLinearDistanceTo(expectedLocation);
                Double db = VisionUtils.getPixelLocation(camera, b.x, b.y)
                        .getLinearDistanceTo(expectedLocation);
                return da.compareTo(db);
            });
            CvStage.Result.Circle closest = results.get(0);
            return VisionUtils.getPixelLocation(camera, closest.x, closest.y);
        }
    }

    // ---- pick location (auto-feeder semantics) ----

    /**
     * Pick location for the auto-feeder.
     * <p>The base point is the sprocket hole currently sitting at the pick
     * station (the most recent vision result), <b>not</b> a position accumulated
     * along the tape. The pick location is therefore:</p>
     * <pre>
     *   pick = currentHole + holeToPartOffset + pickSub * partPitch (along tape)
     * </pre>
     * where {@code pickSub} selects which component within the current hole
     * group is being picked. The tape axis direction is taken from the
     * configured reference/last hole locations.
     */
    @Override
    public Location getPickLocation() throws Exception {
        Location[] lineLocations = getIdealLineLocations();

        // Base hole = the sprocket hole at the pick station. Before the first
        // feed (no vision yet) fall back to the configured reference hole.
        Location baseHole = (visionLocation != null) ? visionLocation : lineLocations[0];

        // Which component within the current hole group are we picking?
        // subFeedIndex is the authoritative component-within-group counter and
        // is incremented at the END of feed(); at pick time it therefore points
        // to the NEXT component, so the one just fed is (subFeedIndex - 1).
        // Deriving pickSub from subFeedIndex (instead of feedCount) keeps the
        // pick location consistent with the advance/no-advance decision even
        // when feedCount gets out of sync (e.g. after a feed-count reset):
        // the advance feed (subFeedIndex was 0) always picks component 0 and the
        // no-advance feed (subFeedIndex was 1) always picks component 1.
        int ppHole = Math.max(1, getPartsPerHole());
        int pickSub = (getFeedCount() == 0) ? 0 : ((subFeedIndex + ppHole - 1) % ppHole);

        // Move from the base hole along the tape axis by pickSub * partPitch.
        Location l = Utils2D.getPointAlongLine(baseHole, lineLocations[1],
                getPartPitch().multiply(pickSub));

        // Add the hole-to-part offset: lateral (perpendicular to tape, derived
        // from tape width) plus the linear (along tape) reference distance, which
        // is 0 for this auto-feeder (see getReferenceHoleToPartLinear()).
        double tapeWidthMm = getTapeWidth().convertToUnits(LengthUnit.Millimeters).getValue();
        Length x = new Length(tapeWidthMm / 2 - 0.5, LengthUnit.Millimeters)
                .convertToUnits(l.getUnits());
        Length y = getReferenceHoleToPartLinear().convertToUnits(l.getUnits());
        Point p = new Point(x.getValue(), y.getValue());

        // Local tape frame is rotated -90° (sprocket holes on the side).
        double angle = Utils2D.getAngleFromPoint(lineLocations[1], lineLocations[0]) - 90;
        p = Utils2D.rotatePoint(p, angle);
        l = l.add(new Location(l.getUnits(), p.x, p.y, 0, 0));

        if (isStandardEia481()) {
            angle += 90;
        }
        l = l.derive(null, null, null, angle + getLocation().getRotation());

        return l;
    }

    @Override
    public boolean supportsFeedOptions() {
        return true;
    }

    /**
     * For the auto-feeder the tape advances by one sprocket-hole pitch so that a
     * hole (and the first component of its group) arrive together at the fixed
     * pick station. Vision therefore locates a hole that is laterally beside
     * component 0 with NO along-tape (linear) offset, so the inherited 2 mm
     * linear offset must be zeroed. Keeping the parent's 2 mm value would push
     * every pick 2 mm along the tape and, for the no-advance sub-feed, send it
     * in the wrong direction.
     */
    @Override
    public Length getReferenceHoleToPartLinear() {
        return new Length(0, LengthUnit.Millimeters);
    }

    @Override
    public String toString() {
        return getName();
    }
}
