package org.openpnp.machine.reference.feeder;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import javax.swing.Action;

import org.openpnp.ConfigurationListener;
import org.openpnp.gui.support.Wizard;
import org.openpnp.machine.reference.ReferenceActuator;
import org.openpnp.machine.reference.ReferenceFeeder;
import org.openpnp.machine.reference.driver.GcodeDriver;
import org.openpnp.machine.reference.feeder.wizards.CassetteFeederConfiguratorConfigurationWizard;
import org.openpnp.model.Configuration;
import org.openpnp.model.Length;
import org.openpnp.model.LengthUnit;
import org.openpnp.model.Location;
import org.openpnp.model.Part;
import org.openpnp.spi.Actuator;
import org.openpnp.spi.Driver;
import org.openpnp.spi.Feeder;
import org.openpnp.spi.Machine;
import org.openpnp.spi.Nozzle;
import org.openpnp.spi.PropertySheetHolder;
import org.simpleframework.xml.Element;
import org.pmw.tinylog.Logger;

/**
 * CassetteFeederConfigurator — the persistent owner of a cassette baseplate's
 * shared layout and the trigger for firmware feeder discovery.
 *
 * <p>A cassette baseplate holds many {@link CassetteAutoFeeder} slots. All of
 * them share the same baseplate geometry (origin offset, row spacing, rail
 * direction, orientation, total rows/cols, …). Storing that shared geometry on
 * each individual feeder both duplicates it and — worse — loses it the moment
 * every feeder is deleted (e.g. when re-discovering into a clean machine).</p>
 *
 * <p>This feeder is therefore the <b>single source of truth</b> for the
 * baseplate layout: it is serialized like any other feeder, it survives the
 * deletion of all {@link CassetteAutoFeeder}s, and its <b>Discover Feeders</b>
 * action uses these parameters to (re)create the per-slot feeders from the
 * firmware report. At least one {@code CassetteFeederConfigurator} must be kept
 * in the machine for {@link CassetteAutoFeeder} to function.</p>
 *
 * <p>It is not a pickable feeder — {@link #feed(Nozzle)} and
 * {@link #getPickLocation()} throw by design.</p>
 */
public class CassetteFeederConfigurator extends ReferenceFeeder {

    public static final String ACTUATOR_DISCOVER_NAME = "CassetteFeederDiscovery";

    // ---- baseplate layout (single source of truth) ----
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

    // ---- stage-1 calibration captures (persisted) ----
    // The slot centers the user captured for the two-point baseplate
    // calibration. Kept on the model so the Calibration panel is repopulated
    // when the wizard is reopened.
    @Element(required = false)
    protected Location calibrationSecondSlot = new Location(LengthUnit.Millimeters, 0, 0, 0, 0);
    @Element(required = false)
    protected Location calibrationSecondLastSlot = new Location(LengthUnit.Millimeters, 0, 0, 0, 0);

    // ---- shared EIA-481 geometry (used by CassetteAutoFeeder too) ----
    public static final double sprocketHoleLateralOffsetMm = 2.0;
    public static final double sprocketHolePitchMm = 4.0;

    public static final Map<Double, Double> tapeWidthToF = new HashMap<>();
    static {
        tapeWidthToF.put(8.0, 3.50);
        tapeWidthToF.put(12.0, 5.50);
        tapeWidthToF.put(16.0, 7.50);
        tapeWidthToF.put(24.0, 11.50);
        tapeWidthToF.put(32.0, 14.20);
        tapeWidthToF.put(44.0, 20.20);
        tapeWidthToF.put(56.0, 26.20);
    }

    public CassetteFeederConfigurator() {
        super();
        Configuration.get().addListener(new ConfigurationListener.Adapter() {
            @Override
            public void configurationLoaded(Configuration configuration) {
                configureActuator();
            }
        });
    }

    @Override
    public Wizard getConfigurationWizard() {
        return new CassetteFeederConfiguratorConfigurationWizard(this);
    }

    @Override
    public PropertySheetHolder[] getChildPropertySheetHolders() {
        return null;
    }

    @Override
    public String getPropertySheetHolderTitle() {
        return getClass().getSimpleName() + " " + getName();
    }

    @Override
    public Action[] getPropertySheetHolderActions() {
        return null;
    }

    // ---- accessors ----
    public Length getBaseplateOffsetX() {
        return baseplateOffsetX;
    }
    public void setBaseplateOffsetX(Length v) {
        Object o = baseplateOffsetX;
        baseplateOffsetX = v;
        firePropertyChange("baseplateOffsetX", o, v);
    }
    public Length getBaseplateOffsetY() {
        return baseplateOffsetY;
    }
    public void setBaseplateOffsetY(Length v) {
        Object o = baseplateOffsetY;
        baseplateOffsetY = v;
        firePropertyChange("baseplateOffsetY", o, v);
    }
    public Length getBaseplateOffsetZ() {
        return baseplateOffsetZ;
    }
    public void setBaseplateOffsetZ(Length v) {
        Object o = baseplateOffsetZ;
        baseplateOffsetZ = v;
        firePropertyChange("baseplateOffsetZ", o, v);
    }
    public Length getRowSpacing() {
        return rowSpacing;
    }
    public void setRowSpacing(Length v) {
        Object o = rowSpacing;
        rowSpacing = v;
        firePropertyChange("rowSpacing", o, v);
    }
    public double getRowDirectionX() {
        return rowDirectionX;
    }
    public void setRowDirectionX(double v) {
        Object o = rowDirectionX;
        rowDirectionX = v;
        firePropertyChange("rowDirectionX", o, v);
    }
    public double getRowDirectionY() {
        return rowDirectionY;
    }
    public void setRowDirectionY(double v) {
        Object o = rowDirectionY;
        rowDirectionY = v;
        firePropertyChange("rowDirectionY", o, v);
    }
    public int getContinuousRowCol() {
        return continuousRowCol;
    }
    public void setContinuousRowCol(int v) {
        Object o = continuousRowCol;
        continuousRowCol = v;
        firePropertyChange("continuousRowCol", o, v);
    }
    public int getSkippedRowCol() {
        return skippedRowCol;
    }
    public void setSkippedRowCol(int v) {
        Object o = skippedRowCol;
        skippedRowCol = v;
        firePropertyChange("skippedRowCol", o, v);
    }
    public boolean getIsVerticalLayout() {
        return isVerticalLayout;
    }
    public void setIsVerticalLayout(boolean v) {
        Object o = isVerticalLayout;
        isVerticalLayout = v;
        firePropertyChange("isVerticalLayout", o, v);
    }
    public int getOrientation() {
        return orientation;
    }
    public void setOrientation(int v) {
        Object o = orientation;
        orientation = v;
        firePropertyChange("orientation", o, v);
    }
    public int getTotalRow() {
        return totalRow;
    }
    public void setTotalRow(int v) {
        Object o = totalRow;
        totalRow = v;
        firePropertyChange("totalRow", o, v);
    }
    public int getTotalCol() {
        return totalCol;
    }
    public void setTotalCol(int v) {
        Object o = totalCol;
        totalCol = v;
        firePropertyChange("totalCol", o, v);
    }
    public Location getCalibrationSecondSlot() {
        return calibrationSecondSlot;
    }
    public void setCalibrationSecondSlot(Location v) {
        Object o = calibrationSecondSlot;
        calibrationSecondSlot = (v == null) ? new Location(LengthUnit.Millimeters, 0, 0, 0, 0) : v;
        firePropertyChange("calibrationSecondSlot", o, calibrationSecondSlot);
    }
    public Location getCalibrationSecondLastSlot() {
        return calibrationSecondLastSlot;
    }
    public void setCalibrationSecondLastSlot(Location v) {
        Object o = calibrationSecondLastSlot;
        calibrationSecondLastSlot = (v == null) ? new Location(LengthUnit.Millimeters, 0, 0, 0, 0) : v;
        firePropertyChange("calibrationSecondLastSlot", o, calibrationSecondLastSlot);
    }

    // ---- shared geometry helpers ----

    /** CCW rotation by the baseplate orientation (0/90/180/270). */
    public static double rotateX(double dx, double dy, int ori) {
        switch (ori) {
            case 90:  return -dy;
            case 180: return -dx;
            case 270: return dy;
            default:  return dx;
        }
    }

    public static double rotateY(double dx, double dy, int ori) {
        switch (ori) {
            case 90:  return dx;
            case 180: return -dy;
            case 270: return -dx;
            default:  return dy;
        }
    }

    /** Inverse of {@link #rotateX}/{@link #rotateY}: recovers (dx,dy) from (a,b)=rotate(dx,dy,ori). */
    public static double inverseRotateX(double a, double b, int ori) {
        switch (ori) {
            case 90:  return b;
            case 180: return -a;
            case 270: return -b;
            default:  return a;
        }
    }

    public static double inverseRotateY(double a, double b, int ori) {
        switch (ori) {
            case 90:  return -a;
            case 180: return -b;
            case 270: return a;
            default:  return b;
        }
    }

    /**
     * Center of slot (row, col) in the machine frame (millimeters), derived from
     * this configurator's baseplate offsets, row spacing and direction. X/Y only
     * — Z is picked up separately (from the captured reference hole / firmware).
     */
    public Location getSlotCenterMm(int row, int col) {
        double rs = rowSpacing.getValue();
        double colDX = isVerticalLayout ? 1.0 : 0.0;
        double colDY = isVerticalLayout ? 0.0 : 1.0;
        double skip = 0;
        if (continuousRowCol > 0) {
            skip = (isVerticalLayout ? (col / continuousRowCol)
                    : (row / continuousRowCol)) * skippedRowCol;
        }
        double sx = baseplateOffsetX.getValue()
                + (row + (isVerticalLayout ? 0 : skip)) * rs * rowDirectionX
                + (col + (isVerticalLayout ? skip : 0)) * rs * colDX;
        double sy = baseplateOffsetY.getValue()
                + (row + (isVerticalLayout ? 0 : skip)) * rs * rowDirectionY
                + (col + (isVerticalLayout ? skip : 0)) * rs * colDY;
        return new Location(LengthUnit.Millimeters, sx, sy, 0, 0);
    }

    // ---- calibration ----

    /**
     * Stage-1 baseplate calibration from two captured slot centers plus the
     * known total number of slots.
     *
     * <p>The very first slot is usually out of the camera's reach (e.g. parked
     * too far to the left of the bed), so the operator instead jogs the camera
     * to the <b>2nd</b> slot and the <b>2nd-last</b> slot and tells us how many
     * slots the rail has in total. Because both captures are the same physical
     * feature of a slot, the vector between them spans exactly
     * {@code totalSlots - 3} slot pitches, from which we derive everything:</p>
     * <pre>
     *   spacing         = |secondLast - second| / (totalSlots - 3)
     *   rowDirection    = (secondLast - second) / |secondLast - second|
     *   baseplateOffset = second - spacing * rowDirection      (extrapolated slot 0)
     *   totalRow        = totalSlots
     *   isVerticalLayout= rail runs mostly along Y (|dy| &gt; |dx|)
     *   baseplateOffsetZ= second.Z
     * </pre>
     *
     * <p>This is the two-point counterpart of the three-point calibration
     * (2nd / 3rd / 2nd-last) used by {@code CassetteFeeder}; it trades the third
     * capture for the slot count. The rail is treated as the "row" axis; the
     * perpendicular "col" axis (relevant only when {@code totalCol &gt; 1}) is
     * left untouched.</p>
     */
    public void calibrateBaseplateFromSlots(Location second, Location secondLast, int totalSlots) {
        if (second == null || secondLast == null) {
            throw new IllegalArgumentException(
                    "Capture the 2nd and 2nd-last slot centers first.");
        }
        second = second.convertToUnits(LengthUnit.Millimeters);
        secondLast = secondLast.convertToUnits(LengthUnit.Millimeters);

        // Persist the captured slot centers so the Calibration panel survives
        // closing/reopening the wizard.
        setCalibrationSecondSlot(second);
        setCalibrationSecondLastSlot(secondLast);

        if (totalSlots < 4) {
            throw new IllegalArgumentException("Need at least 4 slots to calibrate from the 2nd "
                    + "and 2nd-last slot (got " + totalSlots + ").");
        }

        double span = second.getLinearDistanceTo(secondLast);
        if (span < 0.1) {
            throw new IllegalArgumentException("Captured 2nd and 2nd-last slot centers are too "
                    + "close (" + String.format("%.3f", span) + "mm). Re-capture them.");
        }

        // Slot index 1 (2nd) to index totalSlots-2 (2nd-last) spans totalSlots-3 gaps.
        int gaps = totalSlots - 3;
        double spacing = span / gaps;

        double dirX = (secondLast.getX() - second.getX()) / span;
        double dirY = (secondLast.getY() - second.getY()) / span;

        // Extrapolate one spacing backwards from slot index 1 to slot index 0.
        double firstX = second.getX() - spacing * dirX;
        double firstY = second.getY() - spacing * dirY;

        // Rail runs mostly along Y -> flag vertical so the perpendicular "col"
        // axis becomes X (only matters when totalCol > 1).
        boolean vertical = Math.abs(dirY) > Math.abs(dirX);

        setBaseplateOffsetX(new Length(firstX, LengthUnit.Millimeters));
        setBaseplateOffsetY(new Length(firstY, LengthUnit.Millimeters));
        setBaseplateOffsetZ(new Length(second.getZ(), LengthUnit.Millimeters));
        setRowSpacing(new Length(spacing, LengthUnit.Millimeters));
        setRowDirectionX(dirX);
        setRowDirectionY(dirY);
        setIsVerticalLayout(vertical);
        setTotalRow(totalSlots);

        Logger.info("Baseplate calibrated from 2 slot centers: slots={}, origin=({:.3f}, {:.3f}, "
                + "{:.3f})mm, spacing={:.3f}mm, direction=({:.4f}, {:.4f}) ({} layout).",
                totalSlots, firstX, firstY, second.getZ(), spacing, dirX, dirY,
                vertical ? "vertical" : "horizontal");
    }

    // ---- actuator setup ----

    public static Actuator configureActuator() {
        Machine machine = Configuration.get().getMachine();
        Actuator actuator = machine.getActuatorByName(ACTUATOR_DISCOVER_NAME);
        if (actuator != null) {
            return actuator;
        }

        actuator = new ReferenceActuator();
        actuator.setName(ACTUATOR_DISCOVER_NAME);
        for (Driver driver : machine.getDrivers()) {
            if (!(driver instanceof GcodeDriver)) {
                continue;
            }
            GcodeDriver gd = (GcodeDriver) driver;
            try {
                if (gd.getName().toLowerCase().contains("3dplacer")) {
                    gd.setCommand(actuator, GcodeDriver.CommandType.ACTUATOR_READ_COMMAND,
                            "M888 {Value}");
                    gd.setCommand(actuator, GcodeDriver.CommandType.ACTUATE_STRING_COMMAND,
                            "M888 {StringValue}");
                    gd.setCommand(actuator, GcodeDriver.CommandType.ACTUATOR_READ_REGEX,
                            "3DP (?<Value>.*)");
                    actuator.setDriver(driver);
                    break;
                }
            }
            catch (Exception e) {
                Logger.error(e);
            }
        }
        try {
            machine.addActuator(actuator);
        }
        catch (Exception ex) {
            Logger.error(ex);
        }
        return actuator;
    }

    // ---- discovery ----

    /**
     * Query the firmware for all slots on this baseplate and (re)create the
     * matching {@link CassetteAutoFeeder}s using this configurator's layout.
     * Existing feeders (matched by id) are updated in place; slots no longer
     * reported are disabled; new slots are added.
     */
    public void discoverFeeders() {
        Actuator actuator = configureActuator();
        try {
            String response = actuator.read(String.format("TR%d TC%d;", totalRow, totalCol));
            Logger.info("discoverFeeders: {}", response);
            processFeeders(response);
        }
        catch (Exception e) {
            Logger.error("discoverFeeders failed: {}", e);
        }
    }

    private void processFeeders(String response) {
        response = response.replace("3DPlacer", "");
        String[] components = response.split(";");
        Pattern pattern = Pattern.compile("\\b(\\w+):([^,]+)\\b");
        Map<String, Map<String, String>> map = new HashMap<>();
        for (String comp : components) {
            Matcher m = pattern.matcher(comp);
            Map<String, String> info = new HashMap<>();
            while (m.find()) {
                info.put(m.group(1), m.group(2));
            }
            if (!info.isEmpty()) {
                map.put(info.get("id"), info);
            }
        }
        addFeeders(map);
    }

    private void addFeeders(Map<String, Map<String, String>> map) {
        Machine machine = Configuration.get().getMachine();
        Set<String> kept = new HashSet<>();
        List<Feeder> stale = new ArrayList<>();
        // Snapshot the feeder list: replacing a feeder removes/adds entries,
        // which would otherwise ConcurrentModificationException this loop.
        for (Feeder f : new ArrayList<>(machine.getFeeders())) {
            boolean isAuto = f instanceof CassetteAutoFeeder;
            boolean isLoose = f instanceof CassetteLoosePartFeeder;
            if (!isAuto && !isLoose) {
                continue;
            }
            String id = f.getId();
            if (!map.containsKey(id)) {
                f.setEnabled(false);
                continue;
            }
            Map<String, String> info = map.get(id);
            // Firmware subtype: 0 = tape auto feeder, 1 = loose part cassette.
            boolean wantLoose = "1".equals(info.get("st"));
            if (wantLoose && isLoose) {
                ((CassetteLoosePartFeeder) f).applyDiscovery(this, info);
                kept.add(id);
            }
            else if (!wantLoose && isAuto) {
                ((CassetteAutoFeeder) f).applyDiscovery(this, info);
                kept.add(id);
            }
            else {
                // The reported subtype no longer matches this feeder's class
                // (e.g. a slot changed from tape to loose part): recreate it.
                stale.add(f);
            }
        }
        for (Feeder f : stale) {
            machine.removeFeeder(f);
        }
        for (String key : map.keySet()) {
            if (kept.contains(key)) {
                continue;
            }
            Map<String, String> info = map.get(key);
            if (!info.containsKey("c") || !info.containsKey("r")
                    || !info.containsKey("st")) {
                continue;
            }
            try {
                if ("1".equals(info.get("st"))) {
                    CassetteLoosePartFeeder nf = new CassetteLoosePartFeeder();
                    nf.id = key;
                    nf.applyDiscovery(this, info);
                    machine.addFeeder(nf);
                    Logger.info("Added CassetteLoosePartFeeder id:{}, name:{}", key, info.get("n"));
                }
                else {
                    CassetteAutoFeeder nf = new CassetteAutoFeeder();
                    nf.id = key;
                    nf.applyDiscovery(this, info);
                    machine.addFeeder(nf);
                    Logger.info("Added CassetteAutoFeeder id:{}, name:{}", key, info.get("n"));
                }
            }
            catch (Exception e) {
                Logger.error("Failed to add feeder id:{}", key, e);
            }
        }
    }

    /** Resolve a named part, if present. */
    public static Part resolvePart(String name) {
        return Configuration.get().getPart(name);
    }

    // ---- not a pickable feeder ----

    @Override
    public Location getPickLocation() throws Exception {
        throw new UnsupportedOperationException(
                "CassetteFeederConfigurator does not provide parts. Use it to discover "
                        + "and configure CassetteAutoFeeders.");
    }

    @Override
    public void feed(Nozzle nozzle) throws Exception {
        throw new UnsupportedOperationException(
                "CassetteFeederConfigurator does not feed. Use it to discover "
                        + "and configure CassetteAutoFeeders.");
    }

    @Override
    public String toString() {
        return getName();
    }
}
