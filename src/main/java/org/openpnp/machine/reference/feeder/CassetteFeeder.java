package org.openpnp.machine.reference.feeder;

import java.awt.Color;
import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import org.opencv.core.Mat;
import org.opencv.core.RotatedRect;
import org.opencv.imgproc.Imgproc;
import org.openpnp.ConfigurationListener;
import org.openpnp.gui.MainFrame;
import org.openpnp.gui.support.PropertySheetWizardAdapter;
import org.openpnp.gui.support.Wizard;
import org.openpnp.machine.reference.ReferenceActuator;
import org.openpnp.machine.reference.driver.GcodeDriver;
import org.openpnp.machine.reference.feeder.wizards.CassetteFeederConfigurationWizard;
import org.openpnp.machine.reference.feeder.wizards.ReferencePushPullFeederConfigurationWizard;
import org.openpnp.machine.reference.feeder.wizards.ReferencePushPullMotionConfigurationWizard;
import org.openpnp.model.Configuration;
import org.openpnp.model.Length;
import org.openpnp.model.LengthUnit;
import org.openpnp.model.Location;
import org.openpnp.model.Part;
import org.openpnp.model.RegionOfInterest;
import org.openpnp.spi.Actuator;
import org.openpnp.spi.Camera;
import org.openpnp.spi.Machine;
import org.openpnp.spi.Nozzle;
import org.openpnp.spi.base.AbstractActuator;
import org.openpnp.spi.base.AbstractFeeder;
import org.openpnp.util.FeederVisionHelper;
import org.openpnp.util.FeederVisionHelper.PipelineType;
import org.openpnp.util.MovableUtils;
import org.openpnp.util.OpenCvUtils;
import org.openpnp.util.Utils2D;
import org.openpnp.util.VisionUtils;
import org.openpnp.vision.FluentCv;
import org.openpnp.vision.pipeline.CvPipeline;
import org.openpnp.vision.pipeline.CvStage;
import org.pmw.tinylog.Logger;
import org.simpleframework.xml.Element;
import org.openpnp.spi.Driver;
import org.openpnp.spi.Feeder;
import org.openpnp.spi.Head;

public class CassetteFeeder extends ReferencePushPullFeeder {

    // Hole-detection tolerances used by the pipeline-based auto-setup. These are kept
    // generous so the user can use the same pipeline for various tape widths without
    // re-tuning, but they still reject pockets, fiducials and other non-hole features
    // that the pipeline might pick up on a busy cassette baseplate.
    static final double cassetteSprocketHoleToleranceMm = 0.6;
    // Expected lateral (perpendicular) distance from the part pocket to the sprocket-hole
    // line, i.e. the EIA-481 sprocket-to-part offset. We do NOT validate the inter-hole
    // pitch any more: the camera units-per-pixel scaling (driven by the camera Z vs. the
    // actual tape plane) can stretch the measured pitch by 10–20 %, so a hard 4 mm check
    // wrongly rejects valid setups. Instead we find the pair that physically brackets the
    // pick along the tape axis and only sanity-check the lateral offset.
    static final double cassetteMinLateralDistanceMm = 1.0;
    static final double cassetteMaxLateralDistanceMm = 10.0;
    // EIA-481 sprocket-to-pocket relationship used by the forward/inverse geometry.
    //
    // sprocketHolePitchMm   = 4.0 mm = P0, distance between consecutive sprocket holes
    //                           along the tape (used for scale correction, constant).
    // sprocketHoleToCavityMm = F dimension from EIA-481 — distance from sprocket hole
    //                           center to cavity center, varies with tape width:
    //                             8mm tape  → 3.50mm    24mm tape → 11.50mm
    //                            12mm tape  → 5.50mm    32mm tape → 14.20mm
    //                            16mm tape  → 7.50mm
    // sprocketHoleLateralOffsetMm = 2.0 mm = lateral offset from each hole to the pick
    //                                center (half the inter-hole lateral gap, constant).
    //
    //   hole1 = slotCenter + orient(ox - sprocketHoleToCavityMm, oy + sprocketHoleLateralOffsetMm)
    //   hole2 = slotCenter + orient(ox - sprocketHoleToCavityMm, oy - sprocketHoleLateralOffsetMm)
    @Element(required = false)
    protected double tapeWidthMm = 8.0;
    @Element(required = false)
    protected double sprocketHoleToCavityMm = 3.5;
    static final double sprocketHoleLateralOffsetMm = 2.0;

    /** EIA-481 tape width → F dimension (sprocket hole center to cavity center). */
    static final java.util.Map<Double, Double> tapeWidthToF = new java.util.HashMap<>();
    static {
        tapeWidthToF.put(8.0, 3.50);
        tapeWidthToF.put(12.0, 5.50);
        tapeWidthToF.put(16.0, 7.50);
        tapeWidthToF.put(24.0, 11.50);
        tapeWidthToF.put(32.0, 14.20);
        tapeWidthToF.put(44.0, 20.20);
        tapeWidthToF.put(56.0, 26.20);
    }

    public static final String ACTUATOR_DISCOVER_NAME = "CassetteFeederDiscovery";
    @Element(required = false)
    protected Length baseplateOffsetX= new Length(0,LengthUnit.Millimeters);
    @Element(required = false)
    protected Length baseplateOffsetY= new Length(0,LengthUnit.Millimeters);
    @Element(required = false)
    protected Length baseplateOffsetZ= new Length(0,LengthUnit.Millimeters);
    @Element(required = false)
    protected Length rowSpacing= new Length(8,LengthUnit.Millimeters);    
    // Rail direction vector (unit-length components from calibration), used to
    // decompose row index into X/Y contributions when the rail is not perfectly
    // axis-aligned. rowDirectionX*row + rowDirectionY*row = actual offset along rail.
    @Element(required = false)
    public double rowDirectionX = 1.0;
    @Element(required = false)
    public double rowDirectionY = 0.0;
    // Persisted slot center captures for baseplate calibration (public for wizard access)
    @Element(required = false)
    public Location capturedSlotSecond = null;
    @Element(required = false)
    public Location capturedSlotThird = null;
    @Element(required = false)
    public Location capturedSlotSecondLast = null;
    // Per-feeder contact offsets read from the actuator during discovery
    @Element(required = false)
    protected double feederContactOffsetX=0;
    @Element(required = false)
    protected double feederContactOffsetY=0;
    @Element(required = false)
    protected Length feederHeight = new Length(0, LengthUnit.Millimeters);
    protected double contactToHoleOffsetX=0;
    protected double contactToHoleOffsetY=0;
    protected double contactToPickOffsetX=0;
    protected double contactToPickOffsetY=0;

    @Element(required = false)
    protected int continuousRowCol=0;
    @Element(required = false)
    protected int skippedRowCol=0;
    @Element(required = false)
    protected boolean isVerticalLayout=true;
    @Element(required = false)
    protected int orientation=0;
    @Element(required = false)
    protected int row=0;
    @Element(required = false)    
    protected int col=0;
    @Element(required = false)    
    protected int subType=0;    
    @Element(required = false)
    protected int totalRow=30;
    @Element(required = false)    
    protected int totalCol=15;
    public CassetteFeeder(){
        super();

        Configuration.get().addListener(new ConfigurationListener.Adapter() {
            @Override
            public void configurationLoaded(Configuration configuration) {
                // Ensure actuators are added to the machine when it has PhotonFeeders
                configureActuator();
                // Migrate existing feeders that were saved with calibration disabled.
                // The new default is UntilConfident so vision calibration runs during feed.
                if (calibrationTrigger == CalibrationTrigger.None) {
                    calibrationTrigger = CalibrationTrigger.UntilConfident;
                }
            }
        });
        //use circular symmetry for pipeline
        resetPipeline(PipelineType.CircularSymmetry);
        // Enable vision calibration by default: the geometric positions derived from the
        // baseplate model provide a good starting point, but vision refines the actual pick
        // location in case the user hasn't run Auto-Setup or the tape has shifted slightly.
        // Users can set CalibrationTrigger.None if their baseplate is accurate enough.
        setCalibrationTrigger(CalibrationTrigger.UntilConfident);
    }

        @Override
    public Wizard getConfigurationWizard() {
        return new CassetteFeederConfigurationWizard(this);
    }

        @Override
    public PropertySheet[] getPropertySheets() {
        return new PropertySheet[] {
                new PropertySheetWizardAdapter(new ReferencePushPullFeederConfigurationWizard(this), "Configuration"),
                new PropertySheetWizardAdapter(new ReferencePushPullMotionConfigurationWizard(this), "Push-Pull Motion"),
                new PropertySheetWizardAdapter(new CassetteFeederConfigurationWizard(this), "Feeders Setup"),
        };
    }

    public int getRow(){
        return row;
    }
    public void setRow(int val){
        Object oldValue = this.row;
        this.row = val;
        firePropertyChange("row", oldValue, val);
    }   
    public int getCol(){
        return col;
    }
    public void setCol(int val){
        Object oldValue = this.col;
        this.col = val;
        firePropertyChange("col", oldValue, val);
    }

    public int getSubType(){
        return subType;
    }
    public void setSubType(int val){
        Object oldValue = this.subType;
        this.subType = val;
        firePropertyChange("subType", oldValue, val);
    }    

    public int getOrientation(){
        return orientation;
    }
    public int getTotalRow(){
        return totalRow;
    }
    public void setTotalRow(int val){
        Object oldValue = this.totalRow;
        this.totalRow = val;
        firePropertyChange("totalRow", oldValue, val);
    }
    public int getTotalCol(){
        return totalCol;
    }
    public void setTotalCol(int val){
        Object oldValue = this.totalCol;
        this.totalCol = val;
        firePropertyChange("totalCol", oldValue, val);
    }

    public void setOrientation(int val){
        Object oldValue = this.orientation;
        this.orientation = val;
        firePropertyChange("orientation", oldValue, val);
    }

    public double getTapeWidthMm() {
        return tapeWidthMm;
    }

    public void setTapeWidthMm(double val) {
        Object oldWidth = this.tapeWidthMm;
        this.tapeWidthMm = val;
        firePropertyChange("tapeWidthMm", oldWidth, val);
        // Auto-select F dimension from tape width per EIA-481.
        Double f = tapeWidthToF.get(val);
        if (f != null) {
            setSprocketHoleToCavityMm(f);
        }
    }

    public double getSprocketHoleToCavityMm() {
        return sprocketHoleToCavityMm;
    }

    public void setSprocketHoleToCavityMm(double val) {
        Object oldValue = this.sprocketHoleToCavityMm;
        this.sprocketHoleToCavityMm = val;
        firePropertyChange("sprocketHoleToCavityMm", oldValue, val);
    }

    
    public void setRowSpacing(Length val){
        Object oldValue = this.rowSpacing;
        this.rowSpacing = val;
        firePropertyChange("rowSpacing", oldValue, val);
    }

    public Length getRowSpacing(){
        return rowSpacing;
    }

    public double getRowDirectionX() {
        return rowDirectionX;
    }

    public void setRowDirectionX(double val) {
        Object oldValue = this.rowDirectionX;
        this.rowDirectionX = val;
        firePropertyChange("rowDirectionX", oldValue, val);
    }

    public double getRowDirectionY() {
        return rowDirectionY;
    }

    public void setRowDirectionY(double val) {
        Object oldValue = this.rowDirectionY;
        this.rowDirectionY = val;
        firePropertyChange("rowDirectionY", oldValue, val);
    }

    public boolean getIsVerticalLayout(){
        return isVerticalLayout;
    }

    public void setIsVerticalLayout(boolean val){
        Object oldValue = this.isVerticalLayout;
        this.isVerticalLayout = val;
        firePropertyChange("isVerticalLayout", oldValue, val);
    }

    public int getContinuousRowCol(){
        return continuousRowCol;
    }

    public void setContinuousRowCol(int val){
        Object oldValue = this.continuousRowCol;
        this.continuousRowCol = val;
        firePropertyChange("continuousRowCol", oldValue, val);    
    }
    public int getSkippedRowCol(){
        return skippedRowCol;
    }

    public void setSkippedRowCol(int val){
        Object oldValue = this.skippedRowCol;
        this.skippedRowCol = val;
        firePropertyChange("skippedRowCol", oldValue, val);    
    }    

    public Length getBaseplateOffsetX(){
        return baseplateOffsetX;
    }

    public void setBaseplateOffsetX(Length offset){
        Object oldValue = this.baseplateOffsetX;
        this.baseplateOffsetX = offset;
        firePropertyChange("baseplateOffsetX", oldValue, offset);        
    }

    public Length getBaseplateOffsetY(){
        return baseplateOffsetY;
    }

    public void setBaseplateOffsetY(Length offset){
        Object oldValue = this.baseplateOffsetY;
        this.baseplateOffsetY = offset;
        firePropertyChange("baseplateOffsetY", oldValue, offset);        
    }    

    public Length getBaseplateOffsetZ(){
        return baseplateOffsetZ;
    }

    public void setBaseplateOffsetZ(Length offset){
        Object oldValue = this.baseplateOffsetZ;
        this.baseplateOffsetZ = offset;
        firePropertyChange("baseplateOffsetZ", oldValue, offset);        
    }        

    static Actuator configureActuator(){
        Logger.debug("entering config actuator");
        Machine machine = Configuration.get().getMachine();
        Logger.debug("getting machine");
        Actuator actuator = machine.getActuatorByName(ACTUATOR_DISCOVER_NAME);
        Logger.debug("get actuator, name is {}", actuator==null?"null":actuator.getName());
        actuator = createDefaultActuator(machine, actuator);
        return actuator;
    }

    private static Actuator createDefaultActuator(Machine machine,Actuator actuator) {
        if(actuator == null){
            actuator = new ReferenceActuator();
            actuator.setName(ACTUATOR_DISCOVER_NAME);
            Logger.debug("Created actuator");
        }else{
            return actuator;
        }

        for (Driver driver : machine.getDrivers()) {
            if(! (driver instanceof GcodeDriver)) {
                continue;
            }
            GcodeDriver gcodeDriver = (GcodeDriver) driver;
            try {
                //require driver name to have 3DPlacer for auto configuration
                String driverName = gcodeDriver.getName();                    
                Logger.debug("driver name: {}",driverName);

                if(driverName.toLowerCase().contains("3dplacer")){
                    gcodeDriver.setCommand(actuator, GcodeDriver.CommandType.ACTUATOR_READ_COMMAND, "M888 {Value}");
                    gcodeDriver.setCommand(actuator, GcodeDriver.CommandType.ACTUATE_STRING_COMMAND, "M888 {StringValue}");
                    gcodeDriver.setCommand(actuator, GcodeDriver.CommandType.ACTUATOR_READ_REGEX, "3DP (?<Value>.*)");
                    //set driver so that it can get the correct driver.
                    actuator.setDriver(driver);
                    Logger.debug("Setting driver {}",driverName);
                    break;  // Only set this on 1 GCodeDriver
                }
            } catch (Exception e) {
                e.printStackTrace();
            }
        }

        try {
            machine.addActuator(actuator);
        } catch (Exception exception) {
            exception.printStackTrace();
        }
        return actuator;
    }

    public void saveToFeeder(){
        Logger.debug("Entering saveToFeeder");
        Actuator actuator = configureActuator();
        try{
            // Send per-feeder parameters in standard G-code space format.
            // OX/OY are in 0.1mm units.
            String response = actuator.read(String.format(
                "R%d C%d TC%d TR%d PI%d ST%d OX%d OY%d H%d N%s;",
                row, col, totalCol, totalRow,
                (int)(getPartPitch().getValue()*10),
                subType,
                (int)Math.round(feederContactOffsetX * 10),
                (int)Math.round(feederContactOffsetY * 10),
                (int)Math.round(feederHeight.getValue() * 10),
                getPart()==null?getName():getPart().getId()));
            Logger.info("Response of read: {}", response);
        }catch (Exception e) {
            Logger.info("something wrong processing feeders: {}", e);
            e.printStackTrace();
        }
    }

    public void discoverFeeders(){
        Logger.debug("Entering discoverFeeders");
        Actuator actuator = configureActuator();
        try {
            String response = actuator.read(String.format("TR%d TC%d;",totalRow,totalCol));
            Logger.info("Response of read: {}", response);
            processFeeders(response);
        } catch (Exception e) {
            // TODO Auto-generated catch block
            Logger.info("something wrong processing feeders: {}", e);
            e.printStackTrace();
        }
    }
    //Process feeders based on info read from the system
    private void processFeeders(String response) {
        response = response.replace("3DPlacer", "");
        String[] components = response.split(";");
        Pattern pattern = Pattern.compile("\\b(\\w+):([^,]+)\\b");
        Map<String, Map<String,String> > feeders = new HashMap<>();
        for (String component : components) {
            Matcher matcher = pattern.matcher(component);
            Map<String,String> feederInfo = new HashMap<>();
            while (matcher.find()) {
                String key = matcher.group(1);
                String value = matcher.group(2);
                Logger.debug("key:{},val:{}",key,value);
        
                // Store key-value pairs in the map
                feederInfo.put(key, value);
            }
            if(!feederInfo.isEmpty()){
                feeders.put(feederInfo.get("id"), feederInfo);
                Logger.debug("adding feeder id:{}",feederInfo.get("id"));
            }
        }
        Logger.debug("finished adding feeders");
        //add feeder if not exist
        addFeeders(feeders);
    }
    //Add or modify feeders according to the reading from system
    private void addFeeders(Map<String, Map<String, String>> feeders) {
        Logger.debug("In addFeeders");
        Map<String,String> processedFeeders=new HashMap<>();
        Logger.debug("new processed feeder created");
        for (Feeder feeder : Configuration.get().getMachine().getFeeders()) {
            if(!(feeder instanceof CassetteFeeder) && !(feeder instanceof CassetteLoosePartFeeder)){
                continue;
            }
            String id = feeder.getId();
            if(feeders.containsKey(id)){
                //current feeder exists, update the info
                Map<String,String> feederInfo = feeders.get(id);
                Logger.debug("Existing feeder: {}",id);
                if(!(feederInfo.containsKey("c") && 
                    feederInfo.containsKey("r")&& 
                    feederInfo.containsKey("h"))){
                    Logger.warn("Feeder id:{} lack of row,column or height info, skipping..");
                    continue;
                }

                if(feederInfo.containsKey("st") && "0".equals(feederInfo.get("st"))){
                    //if subtype is 0, set it to 1
                CassetteFeeder theFeeder = (CassetteFeeder) feeder;
                updateFeeder(feederInfo, theFeeder);
                }
                if(feederInfo.containsKey("st") && "1".equals(feederInfo.get("st"))){
                    //if subtype is 0, set it to 1
                CassetteLoosePartFeeder theFeeder = (CassetteLoosePartFeeder) feeder;
                updateLoosePartFeeder(feederInfo, theFeeder);
                }


                processedFeeders.put(id,id);
            }else{
                //if not in the system, set feeders to disable
                feeder.setEnabled(false);
            }
        }        
        Logger.debug("Adding new feeders");

        for (String key : feeders.keySet()) {
            if(processedFeeders.containsKey(key)){
                Logger.debug("skipping feeder:{}, already configured",key);                
                continue;
            }
            Map<String,String> feederInfo = feeders.get(key);
            Logger.debug("Adding new feeders {}, st: {}, contain: {}",key, feederInfo.get("st"),("1".equals(feederInfo.get("st"))));
            if(!(feederInfo.containsKey("c") && feederInfo.containsKey("r")&& feederInfo.containsKey("st"))){
                Logger.warn("Feeder id:{} lack of row,column or height info, skipping..");
                continue;
            }

            try {
                if(feederInfo.containsKey("st") && ("0".equals(feederInfo.get("st")))){
                    //if subtype is 0, set it to 1
                    
                    CassetteFeeder newFeeder = new CassetteFeeder();
                    newFeeder.id = key;
                    updateFeeder(feederInfo, newFeeder);
                    
                    Logger.info("Added feeder id:{}, name: {}",key,feederInfo.get("n"));                                    
                    Configuration.get().getMachine().addFeeder(newFeeder);
                }
                if(feederInfo.containsKey("st") && ("1".equals(feederInfo.get("st")))){
                    //if subtype is 0, set it to 1
                    CassetteLoosePartFeeder newFeeder = new CassetteLoosePartFeeder();
                    newFeeder.id = key;
                    updateLoosePartFeeder(feederInfo, newFeeder);
                    Logger.info("Added feeder id:{}, name: {}",key,feederInfo.get("n"));                                    
                    Configuration.get().getMachine().addFeeder(newFeeder);
                }

            } catch (Exception e) {                
                Logger.error("now able to add feeder with id:{}",key);
                e.printStackTrace();
            }
        }
    }

        private void updateLoosePartFeeder(Map<String, String> feederInfo, CassetteLoosePartFeeder theFeeder) {

        int col = Integer.parseInt(feederInfo.get("c"));
        int row = Integer.parseInt(feederInfo.get("r"));
        double ox = feederInfo.containsKey("ox")?Double.parseDouble(feederInfo.get("ox")):0;
        double oy = feederInfo.containsKey("oy")?Double.parseDouble(feederInfo.get("oy")):0;
        // Height from feeder is in 0.1mm units, convert to mm by dividing by 10
        double h = Double.parseDouble(feederInfo.get("h")) / 10.0;
        int subType = feederInfo.containsKey("st")?Integer.parseInt(feederInfo.get("st")):0;

        theFeeder.setCol(col);
        theFeeder.setSubType(subType);
        theFeeder.setRow(row);
        theFeeder.setTotalCol(totalCol);
        theFeeder.setTotalRow(totalRow);
        theFeeder.setBaseplateOffsetZ(baseplateOffsetZ);
        // Store per-feeder height for later offset recalculation
        theFeeder.feederHeight = new Length(h, LengthUnit.Millimeters);
        theFeeder.setLocation(calculatePickLocationFromRowColHeight(row,col,ox,oy,h));
        theFeeder.setName(feederInfo.get("n"));
        Part thePart = Configuration.get().getPart(feederInfo.get("n"));
        if(thePart!=null){
            theFeeder.setPart(thePart);
        }

        theFeeder.setEnabled(true);
    }

    private void updateFeeder(Map<String, String> feederInfo, CassetteFeeder theFeeder) {

        int col = Integer.parseInt(feederInfo.get("c"));
        int row = Integer.parseInt(feederInfo.get("r"));
        double ox = feederInfo.containsKey("ox")?Double.parseDouble(feederInfo.get("ox")):0;
        double oy = feederInfo.containsKey("oy")?Double.parseDouble(feederInfo.get("oy")):0;
        // Height from feeder is in 0.1mm units, convert to mm by dividing by 10
        double h = Double.parseDouble(feederInfo.get("h")) / 10.0;
        double pi = feederInfo.containsKey("pi")?Double.parseDouble(feederInfo.get("pi")):getPartPitch().getValue()*10;
        int subType = feederInfo.containsKey("st")?Integer.parseInt(feederInfo.get("st")):0;

        theFeeder.setCol(col);
        theFeeder.setSubType(subType);
        theFeeder.setRow(row);
        theFeeder.setTotalCol(totalCol);
        theFeeder.setTotalRow(totalRow);
        theFeeder.setRowSpacing(rowSpacing);
        theFeeder.setRowDirectionX(rowDirectionX);
        theFeeder.setRowDirectionY(rowDirectionY);
        // Store per-feeder contact offsets and height from firmware flash
        theFeeder.feederContactOffsetX = ox;
        theFeeder.feederContactOffsetY = oy;
        theFeeder.feederHeight = new Length(h, LengthUnit.Millimeters);
        theFeeder.setHole1Location(calculateHole1LocationFromRowColHeight(row,col,ox,oy,h));
        theFeeder.setHole2Location(calculateHole2LocationFromRowColHeight(row,col,ox,oy,h));
        theFeeder.setLocation(calculatePickLocationFromRowColHeight(row,col,ox,oy,h));
        theFeeder.setName(feederInfo.get("n"));
        Part thePart = Configuration.get().getPart(feederInfo.get("n"));
        if(thePart!=null){
            theFeeder.setPart(thePart);
        }
        theFeeder.setBaseplateOffsetX(baseplateOffsetX);
        theFeeder.setBaseplateOffsetY(baseplateOffsetY);
        theFeeder.setBaseplateOffsetZ(baseplateOffsetZ);
        theFeeder.setContinuousRowCol(continuousRowCol);
        theFeeder.setSkippedRowCol(skippedRowCol);
        theFeeder.setIsVerticalLayout(isVerticalLayout);
        theFeeder.setPartPitch(new Length(pi/10.0, LengthUnit.Millimeters));
        // Note: no OCR region is generated for cassette feeders. Cassette feeders identify
        // their parts through the firmware (the part id is part of the discovery response),
        // so OCR is not needed. Setting a dummy region in machine coordinates would also
        // break the AffineWarp stage of the pipeline (it expects camera-relative coords).
        theFeeder.setEnabled(true);
    }

    /**
     * Recalculates the baseplate offsets (Offset X, Offset Y, Offset Z) from the
     * currently set hole1Location, hole2Location, and pick location, using the stored
     * per-feeder contact offsets and height read from the actuator during discovery.
     * 
     * This allows the user to manually adjust one feeder, then propagate the correction
     * to all other feeders on the same baseplate by updating the shared offset values.
     */
    public void calculateBaseplateOffsetsFromPositions() {
        double rs = rowSpacing.getValue();
        int rowIdx = row;
        int colIdx = col;
        double ox = feederContactOffsetX;
        double oy = feederContactOffsetY;
        double h = feederHeight.getValue();
        double colDX = getColDirectionX(), colDY = getColDirectionY();

        double skipCount = 0;
        if (continuousRowCol > 0) {
            skipCount = (isVerticalLayout ? (colIdx/continuousRowCol) : (rowIdx/continuousRowCol)) * skippedRowCol;
        }
        double rowX = (rowIdx + (isVerticalLayout ? 0 : skipCount)) * rs * rowDirectionX;
        double rowY = (rowIdx + (isVerticalLayout ? 0 : skipCount)) * rs * rowDirectionY;
        double colX = (colIdx + (isVerticalLayout ? skipCount : 0)) * rs * colDX;
        double colY = (colIdx + (isVerticalLayout ? skipCount : 0)) * rs * colDY;

        Location pickLoc = getLocation().convertToUnits(LengthUnit.Millimeters);
        Location hole1Loc = getHole1Location().convertToUnits(LengthUnit.Millimeters);

        // The pick location is the ground truth — the user jogged the camera to the pocket
        // center and captured it. Derive the baseplate offsets from it directly.
        double pickContribX = rowX + colX + getOffsetXWithOrientation(ox, oy, orientation);
        double pickContribY = rowY + colY + getOffsetYWithOrientation(ox, oy, orientation);

        double newOffsetX = pickLoc.getX() - pickContribX;
        double newOffsetY = pickLoc.getY() - pickContribY;
        double newOffsetZ = pickLoc.getZ() - h;

        // Cross-check against hole1 (diagnostic only)
        double hole1ContribX = rowX + colX + getOffsetXWithOrientation(ox - sprocketHoleToCavityMm, oy + 2, orientation);
        double hole1ContribY = rowY + colY + getOffsetYWithOrientation(ox - sprocketHoleToCavityMm, oy + 2, orientation);
        double hole1OffsetX = hole1Loc.getX() - hole1ContribX;
        double hole1OffsetY = hole1Loc.getY() - hole1ContribY;
        double discrepancy = Math.sqrt(
                Math.pow(newOffsetX - hole1OffsetX, 2) + Math.pow(newOffsetY - hole1OffsetY, 2));

        Logger.info("Calculated baseplate offsets (from Pick Location): X="
                + String.format("%.4f", newOffsetX) + "mm, Y="
                + String.format("%.4f", newOffsetY) + "mm, Z="
                + String.format("%.4f", newOffsetZ) + "mm");
        Logger.info("  Cross-check from hole1: X=" + String.format("%.4f", hole1OffsetX)
                + "mm, Y=" + String.format("%.4f", hole1OffsetY)
                + "mm  (discrepancy " + String.format("%.3f", discrepancy) + "mm)");
        if (discrepancy > 0.5) {
            Logger.warn("  Pick/hole1 discrepancy is " + String.format("%.2f", discrepancy)
                    + "mm — hole detection may be off. Positions are derived from the Pick "
                    + "Location, which is trusted over the vision-detected hole.");
            if (discrepancy > 1.5) {
                Logger.warn("  Large discrepancy (>1.5mm) may also indicate the Orientation is "
                        + "wrong. Try flipping Orientation (e.g., 0↔180 or 90↔270) and re-run "
                        + "Auto-Setup. The logged lateral-offset side (left/right of axis) "
                        + "tells you which side the holes were detected on.");
            }
        }

        setBaseplateOffsetX(new Length(newOffsetX, LengthUnit.Millimeters));
        setBaseplateOffsetY(new Length(newOffsetY, LengthUnit.Millimeters));
        setBaseplateOffsetZ(new Length(newOffsetZ, LengthUnit.Millimeters));
    }

    /**
     * Stage 1 calibration: establish the baseplate reference frame manually.
     *
     * The user jogs the camera to the center of three slots and captures each location:
     *   - first  : the slot at the grid origin (row 0 / col 0). Becomes baseplateOffset.
     *   - second : the next slot along the rail. Defines the slot spacing and rail direction.
     *   - last   : the far end of the rail. Defines the total number of slots.
     *
     * From these three points we derive:
     *   baseplateOffsetX/Y/Z = first (the grid origin, on the baseplate reference plane)
     *   rowSpacing           = |second - first|
     *   isVerticalLayout     = the rail runs along Y (|dy| > |dx|) rather than X
     *   totalRow             = round(|last - first| / rowSpacing) + 1
     *
     * The rail is always treated as the "row" axis: with isVerticalLayout=true the rows run
     * along Y, with isVerticalLayout=false they run along X. The perpendicular "col" axis
     * (totalCol) is left for the user to set if the baseplate has more than one rail.
     */

    /**
     * Stage 1 calibration (camera-limited variant): use 2nd, 3rd, and second-last
     * slots when the camera cannot reach the very first or last slot.
     *
     *   - second     : the 2nd slot along the rail (index 1).
     *   - third      : the 3rd slot along the rail (index 2). Defines spacing & direction.
     *   - secondLast : the second-to-last slot (index totalRow-2). Defines total slots.
     *
     * Derivation:
     *   spacing            = |third - second|
     *   rowDirectionX/Y    = (third - second) / spacing
     *   totalRow           = round(|secondLast - second| / spacing) + 3
     *   baseplateOffset    = second - 1 row backwards along the rail direction
     */
    public void calibrateBaseplateFromInnerSlots(Location second, Location third, Location secondLast) {
        second = second.convertToUnits(LengthUnit.Millimeters);
        third = third.convertToUnits(LengthUnit.Millimeters);
        secondLast = secondLast.convertToUnits(LengthUnit.Millimeters);

        double spacing = second.getLinearDistanceTo(third);
        if (spacing < 0.1) {
            throw new IllegalArgumentException("Second and Third slot centers are too close "
                    + "(spacing " + String.format("%.3f", spacing) + "mm). Re-capture them.");
        }
        double dx = third.getX() - second.getX();
        double dy = third.getY() - second.getY();
        double dirX = dx / spacing;
        double dirY = dy / spacing;

        double span = second.getLinearDistanceTo(secondLast);
        int numSlots = (int) Math.round(span / spacing) + 3;  // +3: gap from idx 1 to idx N-2
        if (numSlots < 3) { numSlots = 3; }

        // Extrapolate back to slot 0 (the real first slot)
        double firstX = second.getX() - spacing * dirX;
        double firstY = second.getY() - spacing * dirY;

        boolean vertical = Math.abs(dy) > Math.abs(dx);
        double angleDeg = Math.toDegrees(Math.atan2(dy, dx));

        setBaseplateOffsetX(new Length(firstX, LengthUnit.Millimeters));
        setBaseplateOffsetY(new Length(firstY, LengthUnit.Millimeters));
        setBaseplateOffsetZ(new Length(second.getZ(), LengthUnit.Millimeters));
        setRowSpacing(new Length(spacing, LengthUnit.Millimeters));
        setRowDirectionX(dirX);
        setRowDirectionY(dirY);
        setIsVerticalLayout(vertical);
        setTotalRow(numSlots);

        Logger.info("Baseplate calibrated from inner slots: origin=("
                + String.format("%.3f", firstX) + ", " + String.format("%.3f", firstY)
                + ", " + String.format("%.3f", second.getZ()) + ")mm, spacing="
                + String.format("%.3f", spacing) + "mm, rail angle="
                + String.format("%.2f", angleDeg) + "° (" + (vertical ? "vertical/Y" : "horizontal/X")
                + "), direction=(" + String.format("%.4f", dirX) + ", " + String.format("%.4f", dirY) + ")"
                + "), totalRow=" + numSlots + " slots.");
    }

    /**
     * Vision-based baseplate calibration: scan each slot from 2nd to 2nd-last using the
     * SlotRectangles pipeline to detect the 5-rectangle slot pattern. Records all slot
     * centers and uses 2nd, 3rd, and 2nd-last for calibration.
     */
    public void calibrateBaseplateFromSlots(Location first, Location second, Location last) {
        first = first.convertToUnits(LengthUnit.Millimeters);
        second = second.convertToUnits(LengthUnit.Millimeters);
        last = last.convertToUnits(LengthUnit.Millimeters);

        double spacing = first.getLinearDistanceTo(second);
        if (spacing < 0.1) {
            throw new IllegalArgumentException("First and Second slot centers are too close "
                    + "(spacing " + String.format("%.3f", spacing) + "mm). Re-capture them.");
        }
        double span = first.getLinearDistanceTo(last);
        int numSlots = (int) Math.round(span / spacing) + 1;
        if (numSlots < 2) {
            numSlots = 2;
        }

        double dx = second.getX() - first.getX();
        double dy = second.getY() - first.getY();
        boolean vertical = Math.abs(dy) > Math.abs(dx);
        double angleDeg = Math.toDegrees(Math.atan2(dy, dx));

        // Store unit direction vector of the rail (normalize dx,dy by spacing)
        double dirX = dx / spacing;
        double dirY = dy / spacing;

        setBaseplateOffsetX(new Length(first.getX(), LengthUnit.Millimeters));
        setBaseplateOffsetY(new Length(first.getY(), LengthUnit.Millimeters));
        setBaseplateOffsetZ(new Length(first.getZ(), LengthUnit.Millimeters));
        setRowSpacing(new Length(spacing, LengthUnit.Millimeters));
        setRowDirectionX(dirX);
        setRowDirectionY(dirY);
        setIsVerticalLayout(vertical);
        setTotalRow(numSlots);

        Logger.info("Baseplate calibrated from 3 slots: origin=("
                + String.format("%.3f", first.getX()) + ", " + String.format("%.3f", first.getY())
                + ", " + String.format("%.3f", first.getZ()) + ")mm, spacing="
                + String.format("%.3f", spacing) + "mm, rail angle="
                + String.format("%.2f", angleDeg) + "° (" + (vertical ? "vertical/Y" : "horizontal/X")
                + "), direction=(" + String.format("%.4f", dirX) + ", " + String.format("%.4f", dirY) + ")"
                + "), totalRow=" + numSlots + " slots.");
    }

    /**
     * Stage 2 calibration: derive the per-feeder contact offset (ox, oy) from the
     * known sprocket-hole and pick locations.
     *
     * Hole 1 and Pick Location must already be set -- Auto-Setup does both at once
     * (it detects the sprocket holes and derives the pick from the EIA-481 geometry:
     * pick = hole1 - orient(4, 2)).  Alternatively the user can capture each manually.
     *
     * The contact offset (ox, oy) is computed DIRECTLY from the hole1-to-pick vector,
     * not via the slot center, so the result is orientation-safe and independent of
     * Stage 1 baseplate calibration accuracy:
     *
     *   local = unRotate(hole1 - pick, orientation) = (ox + 4, oy + 2)
     *   ox = local.x - sprocketHolePitchMm
     *   oy = local.y - sprocketHoleLateralOffsetMm
     *
     * The pick location and hole2 are then re-derived from the baseplate model so
     * that the Z and orientation are consistent.  The resulting ox/oy (and feederHeight)
     * can be saved to the feeder firmware with saveToFeeder().
     */
    public void calibrateFeederOffsetFromHole1() throws Exception {
        Location hole1 = getHole1Location();
        if (hole1 == null || !hole1.multiply(1, 1, 0, 0).isInitialized()) {
            throw new Exception("Hole 1 Location is not set. Detect it with Auto-Setup / Preview, "
                    + "or jog the camera to a sprocket hole and click the Capture button next to "
                    + "Hole 1 Location, then press Calibrate Feeder Offset.");
        }
        hole1 = hole1.convertToUnits(LengthUnit.Millimeters);

        // Retain the current pick location for the discrepancy warning (it may be unset
        // or far off — we will replace it with the geometry-derived position).
        Location capturedPick = getLocation().convertToUnits(LengthUnit.Millimeters);

        // ------------------------------------------------------------------
        // Compute the slot center position from the baseplate model (without ox/oy).
        // Uses the rail direction vector so tilted baseplates are handled correctly.
        // ------------------------------------------------------------------
        double rs = rowSpacing.getValue();
        double colDX = getColDirectionX(), colDY = getColDirectionY();
        double skipCount = 0;
        if (continuousRowCol > 0) {
            skipCount = (isVerticalLayout ? (col/continuousRowCol) : (row/continuousRowCol)) * skippedRowCol;
        }
        double rowX = (row + (isVerticalLayout ? 0 : skipCount)) * rs * rowDirectionX;
        double rowY = (row + (isVerticalLayout ? 0 : skipCount)) * rs * rowDirectionY;
        double colX = (col + (isVerticalLayout ? skipCount : 0)) * rs * colDX;
        double colY = (col + (isVerticalLayout ? skipCount : 0)) * rs * colDY;
        double slotCenterX = baseplateOffsetX.getValue() + rowX + colX;
        double slotCenterY = baseplateOffsetY.getValue() + rowY + colY;

        double hole1FromSlotX = hole1.getX() - slotCenterX;
        double hole1FromSlotY = hole1.getY() - slotCenterY;
        double[] unRotated = unRotateOffset(hole1FromSlotX, hole1FromSlotY, orientation);
        double ox = unRotated[0] + sprocketHoleToCavityMm;
        double oy = unRotated[1] - sprocketHoleLateralOffsetMm;
        feederContactOffsetX = ox;
        feederContactOffsetY = oy;

        // ------------------------------------------------------------------
        // Derive both hole2 and pick from hole1 using fixed EIA-481 geometry.
        // The two sprocket holes are 2*sprocketHoleLateralOffsetMm (4 mm) apart
        // perpendicular to the tape axis, and the pick sits on the component
        // side of the hole line, centered between them.
        //
        //   hole2 = hole1 - orient(0, 4)     (4 mm lateral gap)
        //   pick  = hole1 + orient(sprocketHoleToCavityMm, -sprocketHoleLateralOffsetMm)
        //
        // At orientation 0 (hole1 at higher Y than hole2):
        //   hole2 = (hole1.x, hole1.y - 4)     — hole1 above hole2
        //   pick  = (hole1.x + 4, hole1.y - 2) — component to the RIGHT
        // ------------------------------------------------------------------

        LengthUnit su = Configuration.get().getSystemUnits();
        // Use hole1 Z for the pick and hole2 (they are on the same tape plane).
        double holeZ = hole1.getLengthZ().getValue();

        // Height: feeder height above the baseplate reference Z.
        double h = holeZ - baseplateOffsetZ.getValue();
        feederHeight = new Length(h, LengthUnit.Millimeters);

        // hole2 from hole1
        double lateralGapMm = 2 * sprocketHoleLateralOffsetMm;
        double hole2X = hole1.getX() + getOffsetXWithOrientation(0, -lateralGapMm, orientation);
        double hole2Y = hole1.getY() + getOffsetYWithOrientation(0, -lateralGapMm, orientation);
        setHole2Location(new Location(LengthUnit.Millimeters, hole2X, hole2Y, holeZ, 0)
                .convertToUnits(su));

        // pick from hole1 (component to the right at ori 0, left at ori 180)
        double pickDerivedX = hole1.getX() + getOffsetXWithOrientation(
                sprocketHoleToCavityMm, -sprocketHoleLateralOffsetMm, orientation);
        double pickDerivedY = hole1.getY() + getOffsetYWithOrientation(
                sprocketHoleToCavityMm, -sprocketHoleLateralOffsetMm, orientation);
        Location derivedPick = new Location(LengthUnit.Millimeters,
                pickDerivedX, pickDerivedY, holeZ, capturedPick.getRotation());
        if (capturedPick.multiply(1, 1, 0, 0).isInitialized()) {
            double pickDiscrepancy = Math.sqrt(
                    Math.pow(pickDerivedX - capturedPick.getX(), 2)
                    + Math.pow(pickDerivedY - capturedPick.getY(), 2));
            if (pickDiscrepancy > 0.5) {
                Logger.warn("Calibrate Feeder Offset: captured pick ("
                        + String.format("%.3f", capturedPick.getX()) + ", "
                        + String.format("%.3f", capturedPick.getY()) + ") differs from "
                        + "geometry-derived pick by " + String.format("%.2f", pickDiscrepancy)
                        + "mm. Replacing with derived pick ("
                        + String.format("%.3f", pickDerivedX) + ", "
                        + String.format("%.3f", pickDerivedY) + ").");
            }
        }
        setLocation(derivedPick);

        Logger.info("Feeder offset calibrated from hole1: ox=" + String.format("%.3f", ox)
                + "mm, oy=" + String.format("%.3f", oy) + "mm, h="
                + String.format("%.3f", h) + "mm (slot (" + row + "," + col
                + "), orientation " + orientation + ").");
        Logger.info("  hole1=(" + String.format("%.3f", hole1.getX()) + ", "
                + String.format("%.3f", hole1.getY()) + ")mm, pick=("
                + String.format("%.3f", pickDerivedX) + ", " + String.format("%.3f", pickDerivedY)
                + ")mm, hole2=(" + String.format("%.3f", hole2X) + ", "
                + String.format("%.3f", hole2Y) + ")mm.");
    }

    /**
     * Inverse of getOffsetX/YWithOrientation: rotate a machine-space delta back into
     * feeder-local coordinates for the given orientation (0/90/180/270).
     */
    private double[] unRotateOffset(double dX, double dY, int ori) {
        switch (ori) {
            case 90:
                return new double[] { -dY, dX };
            case 180:
                return new double[] { -dX, -dY };
            case 270:
                return new double[] { dY, -dX };
            default:
                return new double[] { dX, dY };
        }
    }

    /**
     * Resolves the pick location Z from the baseplate model if currently uninitialized.
     * Uses baseplateOffsetZ + feederHeight. Falls back to current camera Z if model Z is 0.
     */
    private void ensurePickZ(Camera camera) throws Exception {
        if (getLocation().getLengthZ().isInitialized()) {
            return; // already set
        }
        double modelZ = baseplateOffsetZ.add(feederHeight).getValue();
        if (modelZ > 0.001) {
            setLocation(getLocation().deriveLengths(null, null, 
                new Length(modelZ, LengthUnit.Millimeters), null));
            Logger.info("Set pick Z from baseplate model: " + modelZ + "mm");
        } else if (camera.getLocation().getLengthZ().isInitialized()) {
            double cameraZ = camera.getLocation().getLengthZ().getValue();
            setLocation(getLocation().deriveLengths(null, null, 
                new Length(cameraZ, LengthUnit.Millimeters), null));
            Logger.info("Set pick Z from camera Z: " + cameraZ + "mm");
        }
    }

    /**
     * Overrides autoSetup for CassetteFeeder. Uses the user-specified pick location as the
     * reference, runs the configurable vision pipeline to detect sprocket-hole circles,
     * filters them by diameter, and picks the pair of holes that are at the nominal 4 mm
     * sprocket pitch and whose midpoint is closest to the pick location.
     *
     * Workflow:
     *   1. Jog the camera to the center of the part pocket.
     *   2. Click the "Capture Camera Location" button next to Pick Location.
     *   3. (optional) Tune the pipeline with "Preview Vision Features" / "Edit Pipeline".
     *   4. Click "Auto-Setup".
     *
     * The pick location is taken as-is (it is what the user captured). Only the hole 1 / hole 2
     * locations and the baseplate offsets are computed.
     */
    @Override
    public void autoSetup() throws Exception {
        Camera camera = getCamera();

        // The pick location must be set by the user (jog camera to the pocket and capture).
        Location pickLoc = getLocation().convertToUnits(LengthUnit.Millimeters);
        if (!pickLoc.multiply(1, 1, 0, 0).isInitialized()) {
            throw new Exception("Pick Location is not set. Jog the camera to the center of the "
                    + "part pocket, click the Capture Camera Location button next to Pick Location, "
                    + "then press Auto-Setup.");
        }

        // Make sure Z is populated before we move.
        ensurePickZ(camera);
        ensureCameraZ(camera, true);
        // Move the camera to the pick location so the sprocket holes flank the pocket and
        // are well inside the field of view.
        MovableUtils.moveToLocationAtSafeZ(camera, pickLoc.derive(null, null, null, 0.0));

        // performOcr = false: cassette feeders identify parts through firmware, and we don't
        // want a stale OCR region to trigger the AffineWarp stage.
        try (CvPipeline pipeline = getCvPipeline(camera, true, false, true)) {
            pipeline.process();

            List<CvStage.Result.Circle> circles = extractCirclesFromPipeline(pipeline);

            // Keep only circles whose diameter is close to the nominal sprocket-hole diameter.
            List<Location> holes = filterHolesByDiameter(camera, circles);
            Logger.info("Auto-Setup: pipeline detected " + circles.size() + " circle(s), "
                    + holes.size() + " passed the ~" + sprocketHoleDiameterMm + "mm diameter filter.");
            for (int i = 0; i < holes.size(); i++) {
                Location hh = holes.get(i);
                Logger.info("Auto-Setup:   hole[" + i + "]=("
                        + String.format("%.3f", hh.getX()) + ", " + String.format("%.3f", hh.getY())
                        + ") mm, distance from pick="
                        + String.format("%.3f", hh.getLinearDistanceTo(pickLoc)) + "mm");
            }

            // Find the pair of holes at nominal 4 mm pitch whose midpoint is on the expected
            // ~4 mm ring around the pick location. This is far more stable than "two nearest
            // holes" because it validates the geometric relationship between the two holes.
            Location[] pair = findBestSprocketHolePair(holes, pickLoc);
            Location hole1 = pair[0];
            Location hole2 = pair[1];

            // Tape angle from the hole vector.
            Location unitVector = hole1.unitVectorTo(hole2);
            double angleTape = Math.atan2(unitVector.getY(), unitVector.getX()) * 180.0 / Math.PI;

            // Order the holes so that hole1 is "above" hole2 (higher Y at ori 0, higher X
            // at ori 90, etc.). The expected direction from hole2 to hole1 is orient(0, 4).
            double expectedDx = getOffsetXWithOrientation(0, 4, orientation);
            double expectedDy = getOffsetYWithOrientation(0, 4, orientation);
            double actualDx = hole1.getX() - hole2.getX();
            double actualDy = hole1.getY() - hole2.getY();
            if (actualDx * expectedDx + actualDy * expectedDy < 0) {
                Location swap = hole2;
                hole2 = hole1;
                hole1 = swap;
            }

            // IMPORTANT: record the holes at their raw detected X/Y positions. Do NOT snap the
            // inter-hole distance to 4 mm — snapping moves the recorded holes away from where
            // they were actually detected, so "move camera to hole" would land beside the hole.
            //
            // Z: the sprocket holes are on the same physical plane as the part pocket, so their
            // Z equals the pick-location Z (the vision height). VisionUtils.getPixelLocation()
            // derives Z from camera.getLocation(), which on some machines reports 0 during
            // pipeline processing instead of the actual camera Z — so we explicitly set the
            // hole Z from the pick location. Without this, "move camera to hole" drops the
            // camera to Z=0 instead of the vision height.
            double measuredPitchMm = hole1.getLinearDistanceTo(hole2);
            Location midPoint = hole1.add(hole2).multiply(0.5, 0.5, 0, 0);
            double midDistMm = midPoint.getLinearDistanceTo(pickLoc);
            Logger.info("Auto-Setup: pair selected. hole1=("
                    + String.format("%.3f", hole1.getX()) + ", " + String.format("%.3f", hole1.getY())
                    + ") hole2=(" + String.format("%.3f", hole2.getX()) + ", "
                    + String.format("%.3f", hole2.getY()) + ") pitch="
                    + String.format("%.3f", measuredPitchMm) + "mm midpointDistFromPick="
                    + String.format("%.3f", midDistMm) + "mm angle="
                    + String.format("%.2f", angleTape) + "°");

            // ---- Scale correction using the known 4mm sprocket pitch. ----
            // Scale all detected positions around the camera's ACTUAL location at the time
            // the image was captured.  This is the correct reference because
            // VisionUtils.getPixelLocation() = camera.getLocation() + pixelOffset, so every
            // detected hole's offset from the camera centre is subject to the same UPP
            // scaling error.  The camera centre is the zero-distortion point.
            //
            // Do NOT scale around pickLoc (user-captured pick may be off) nor around the
            // pair's midpoint (fixes relative spacing but not absolute position).
            Location cameraLocForScale = camera.getLocation()
                    .convertToUnits(LengthUnit.Millimeters)
                    .derive(null, null, 0.0, null);  // Z = 0 for 2D vector ops

            // Filter the holes list to inliers only for scale-gap computation.
            // The raw list may still contain outlier circles that were excluded
            // from bracketing; their spurious projection would corrupt the average.
            List<Location> inlierHoles = new ArrayList<>();
            for (Location h : holes) {
                if (perpendicularDistance(h, hole1, hole2) < 1.0) {
                    inlierHoles.add(h);
                }
            }
            double avgGapMm = computeAverageSprocketGap(inlierHoles, hole1, hole2);
            if (avgGapMm > 0) {
                double scaleFactor = sprocketHolePitchMm / avgGapMm;
                if (scaleFactor >= 0.7 && scaleFactor <= 1.3) {
                    Logger.info("Auto-Setup: camera scale correction: avg measured gap = "
                            + String.format("%.3f", avgGapMm) + "mm, scaling by factor "
                            + String.format("%.4f", scaleFactor)
                            + " (nominal pitch " + sprocketHolePitchMm + "mm).");

                    hole1 = scaleOffsetFromRef(hole1, cameraLocForScale, scaleFactor);
                    hole2 = scaleOffsetFromRef(hole2, cameraLocForScale, scaleFactor);

                    for (int i = 0; i < holes.size(); i++) {
                        holes.set(i, scaleOffsetFromRef(holes.get(i), cameraLocForScale, scaleFactor));
                    }
                }
                else {
                    Logger.warn("Auto-Setup: scale correction factor "
                            + String.format("%.3f", scaleFactor)
                            + " outside safe [0.7, 1.3] range; correction skipped.");
                }
            }

            // ---- Derive the pick location from the detected hole1. ----
            // The cassette geometry is known: in local (feeder) coordinates,
            // hole1 = pick + orient(-sprocketHoleToCavityMm, sprocketHoleLateralOffsetMm).
            // So we calculate pick = hole1 − orient(-sprocketHoleToCavityMm, sprocketHoleLateralOffsetMm, orientation).
            // This is far more reliable than relying on the user to manually centre
            // the camera on a (often microscopic) part pocket. The user only needs
            // to get the camera close enough that the sprocket holes are visible.
            double holeToPickDx = getOffsetXWithOrientation(-sprocketHoleToCavityMm, 2, orientation)
                                - getOffsetXWithOrientation(0, 0, orientation);
            double holeToPickDy = getOffsetYWithOrientation(-sprocketHoleToCavityMm, 2, orientation)
                                - getOffsetYWithOrientation(0, 0, orientation);
            Location derivedPick = hole1.subtract(
                    new Location(LengthUnit.Millimeters, holeToPickDx, holeToPickDy, 0, 0));
            double pickError = derivedPick.getLinearDistanceTo(pickLoc);
            if (pickError > 0.5) {
                Logger.warn("Auto-Setup: geometry-derived pick ("
                        + String.format("%.3f", derivedPick.getX()) + ", "
                        + String.format("%.3f", derivedPick.getY()) + ") differs from "
                        + "captured pick by " + String.format("%.2f", pickError) + "mm. "
                        + "Using derived pick. If this is incorrect, check the Orientation "
                        + "setting and re-capture the pick on a part pocket.");
                pickLoc = derivedPick.derive(null, null, null, pickLoc.getRotation());
                pickLoc = pickLoc.deriveLengths(null, null, pickLoc.getLengthZ(), null);
            }
            else {
                Logger.info("Auto-Setup: captured pick matches geometry-derived pick ("
                        + String.format("%.2f", pickError) + "mm difference).");
            }

            // Explicitly set the hole Z from the (now-ensured) pick-location Z.
            // We use getLocation() rather than the pickLoc snapshot because ensurePickZ()
            // may have populated the Z after the snapshot was taken.
            Length holeZ = getLocation().getLengthZ();
            hole1 = hole1.deriveLengths(null, null, holeZ, null);
            hole2 = hole2.deriveLengths(null, null, holeZ, null);

            setHole1Location(hole1);
            setHole2Location(hole2);
            // Keep the user-specified pick location, just update the rotation to the tape angle.
            setLocation(pickLoc.derive(null, null, null, angleTape));

            // Now that pick + holes are all set, derive the shared baseplate offsets.
            if (getHole1Location().isInitialized() && getHole2Location().isInitialized()
                    && getLocation().isInitialized()) {
                calculateBaseplateOffsetsFromPositions();
            }

            // Show the result on the camera view (cyan = selected pair, green = other holes,
            // yellow crosshair = pick location).
            showDetectedFeatures(camera, pipeline, holes, hole1, hole2, pickLoc, 2000);

            // Move the camera back to the (updated) pick location.
            MovableUtils.moveToLocationAtSafeZ(camera, getLocation());
            MovableUtils.fireTargetedUserAction(camera);
        }
    }

    /**
     * Preview the vision pipeline output without changing any feeder settings. Useful while
     * tuning the pipeline (Edit Pipeline) or verifying that the pick location is centered
     * on a pocket. If a valid hole pair can be found it is highlighted in cyan.
     */
    @Override
    public void showFeatures() throws Exception {
        Camera camera = getCamera();
        ensureCameraZ(camera, true);

        Location pickLoc = getLocation().convertToUnits(LengthUnit.Millimeters);
        boolean pickSet = pickLoc.multiply(1, 1, 0, 0).isInitialized();
        if (pickSet) {
            MovableUtils.moveToLocationAtSafeZ(camera, pickLoc.derive(null, null, null, 0.0));
        }

        try (CvPipeline pipeline = getCvPipeline(camera, true, false, true)) {
            pipeline.process();
            List<CvStage.Result.Circle> circles = extractCirclesFromPipeline(pipeline);
            List<Location> holes = filterHolesByDiameter(camera, circles);
            Logger.info("Preview: " + circles.size() + " circle(s) detected, " + holes.size()
                    + " passed the diameter filter.");
            for (int i = 0; i < holes.size(); i++) {
                Location hh = holes.get(i);
                Logger.info("Preview:   hole[" + i + "]=("
                        + String.format("%.3f", hh.getX()) + ", " + String.format("%.3f", hh.getY())
                        + ") mm" + (pickSet ? ", distance from pick="
                        + String.format("%.3f", hh.getLinearDistanceTo(pickLoc)) + "mm" : ""));
            }

            Location[] pair = null;
            if (holes.size() >= 2 && pickSet) {
                try {
                    pair = findBestSprocketHolePair(holes, pickLoc);
                }
                catch (Exception ex) {
                    Logger.debug("Preview: no valid sprocket-hole pair found: " + ex.getMessage());
                }
            }
            Location hole1 = (pair != null) ? pair[0] : null;
            Location hole2 = (pair != null) ? pair[1] : null;
            showDetectedFeatures(camera, pipeline, holes, hole1, hole2, pickLoc, 3000);
        }
    }

    /**
     * Pull all detected circles/ellipses/keypoints out of the pipeline results and convert them
     * to {@link CvStage.Result.Circle} objects. This mirrors what {@code FeederVisionHelper}
     * does, so both the {@code CircularSymmetry} pipeline (which yields circles) and the
     * {@code ColorKeyed} pipeline (which yields rotated rects / ellipses) work here.
     */
    @SuppressWarnings("unchecked")
    private List<CvStage.Result.Circle> extractCirclesFromPipeline(CvPipeline pipeline)
            throws Exception {
        List<?> raw = pipeline
                .getExpectedResult(VisionUtils.PIPELINE_RESULTS_NAME)
                .getExpectedModel(List.class);
        List<CvStage.Result.Circle> circles = new ArrayList<>();
        if (raw == null) {
            return circles;
        }
        for (Object result : raw) {
            if (result instanceof CvStage.Result.Circle) {
                circles.add((CvStage.Result.Circle) result);
            }
            else if (result instanceof org.opencv.core.RotatedRect) {
                org.opencv.core.RotatedRect rect = (org.opencv.core.RotatedRect) result;
                double diameter = (rect.size.width + rect.size.height) / 2.0;
                circles.add(new CvStage.Result.Circle(rect.center.x, rect.center.y, diameter));
            }
            else if (result instanceof org.opencv.core.KeyPoint) {
                org.opencv.core.KeyPoint kp = (org.opencv.core.KeyPoint) result;
                circles.add(new CvStage.Result.Circle(kp.pt.x, kp.pt.y, kp.size));
            }
        }
        return circles;
    }

    /**
     * Filter raw pipeline circles, keeping only those whose diameter is within tolerance of
     * the nominal sprocket-hole diameter. Returns the surviving circles as machine
     * locations in millimeters.
     */
    private List<Location> filterHolesByDiameter(Camera camera, List<CvStage.Result.Circle> circles) {
        Location mmScale = camera.getUnitsPerPixelAtZ().convertToUnits(LengthUnit.Millimeters);
        List<Location> holes = new ArrayList<>();
        if (circles == null) {
            return holes;
        }
        for (CvStage.Result.Circle c : circles) {
            double diameterMm = c.diameter * mmScale.getX();
            if (Math.abs(diameterMm - sprocketHoleDiameterMm) <= cassetteSprocketHoleToleranceMm) {
                holes.add(VisionUtils.getPixelLocation(camera, c.x, c.y)
                        .convertToUnits(LengthUnit.Millimeters));
            }
            else {
                Logger.debug("Auto-Setup: dismissed circle at (" + c.x + "," + c.y
                        + ") with diameter " + String.format("%.3f", diameterMm)
                        + "mm (tolerance ±" + cassetteSprocketHoleToleranceMm + "mm)");
            }
        }
        return holes;
    }

    /**
     * Pick the two sprocket holes that flank the part pocket.
     *
     * <p>The detection pipeline reports the holes in pixel/machine coordinates. Due to the
     * camera units-per-pixel scaling (which depends on the camera Z versus the actual tape
     * plane) the <em>measured</em> inter-hole pitch can be stretched by 10–20 % relative to
     * the nominal 4 mm. So we deliberately do <strong>not</strong> validate the pitch.
     * Instead we use the physically correct criterion:</p>
     *
     * <ul>
     *   <li>The sprocket holes lie on a straight line (the tape axis). We estimate that axis
     *       from the two holes that are farthest apart (most stable direction).</li>
     *   <li>The part pocket sits laterally offset from the hole line (the EIA-481
     *       sprocket-to-part distance) and <em>between</em> two consecutive sprocket holes
     *       along the tape axis.</li>
     *   <li>So we project every hole and the pick onto the axis, sort the holes by their
     *       projection, and take the two consecutive holes whose projections bracket the
     *       pick's projection. We sanity-check the lateral offset to catch the case where the
     *       pick was captured on the hole line itself.</li>
     * </ul>
     */
    private Location[] findBestSprocketHolePair(List<Location> holes, Location pickLoc) throws Exception {
        int n = holes.size();
        if (n < 2) {
            throw new Exception("At least two sprocket holes need to be detected, found "
                    + n + ". Use \"Preview Vision Features\" and tune the pipeline "
                    + "(\"Edit Pipeline\") if needed.");
        }

        // All work in millimetres.
        Location pick = pickLoc.convertToUnits(LengthUnit.Millimeters);
        List<Location> mm = new ArrayList<>();
        for (Location h : holes) {
            mm.add(h.convertToUnits(LengthUnit.Millimeters));
        }

        // --- Only one possible pair: return it (with a lateral sanity check). ---
        if (n == 2) {
            double lateral = perpendicularDistance(pick, mm.get(0), mm.get(1));
            Logger.info("Auto-Setup: only two holes detected, using them as the pair "
                    + "(lateral offset " + String.format("%.3f", lateral) + "mm).");
            warnIfLateralOutOfRange(lateral);
            return new Location[]{ mm.get(0), mm.get(1) };
        }

        // --- Estimate the tape axis from the best colinear pair of holes. ---
        // Instead of simply picking the farthest-apart pair (which is vulnerable to a single
        // outlier), we try every pair, count how many OTHER holes lie within 1 mm of their
        // line, and pick the pair with the most inliers.  The true sprocket holes are
        // colinear; an outlier hole sits on its own.  Tie-break by choosing the pair with
        // the largest span (most stable direction).
        int iBestA = 0, iBestB = 1;
        int bestInliers = -1;
        double bestSpan = 0;
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                Location hi = mm.get(i);
                Location hj = mm.get(j);
                int inliers = 0;
                for (int k = 0; k < n; k++) {
                    if (k == i || k == j) {
                        continue;
                    }
                    double pd = perpendicularDistance(mm.get(k), hi, hj);
                    // A true sprocket hole sits within ~1 mm of the line.
                    if (pd < 1.0) {
                        inliers++;
                    }
                }
                double span = hi.getLinearDistanceTo(hj);
                if (inliers > bestInliers || (inliers == bestInliers && span > bestSpan)) {
                    bestInliers = inliers;
                    bestSpan = span;
                    iBestA = i;
                    iBestB = j;
                }
            }
        }
        Location axisOrigin = mm.get(iBestA);
        Location axisEnd   = mm.get(iBestB);
        double axisDx = axisEnd.getX() - axisOrigin.getX();
        double axisDy = axisEnd.getY() - axisOrigin.getY();
        double axisLen = Math.hypot(axisDx, axisDy);
        double ux = axisDx / axisLen;
        double uy = axisDy / axisLen;
        double angleTapeAxis = Math.toDegrees(Math.atan2(uy, ux));

        // ---- Filter: keep only holes that are inliers of this axis (< 1 mm). ----
        // Outlier holes (false positives from the pipeline) are excluded from the
        // projection, bracketing and user-visible log.
        boolean[] isInlier = new boolean[n];
        int nInliers = 0;
        for (int i = 0; i < n; i++) {
            double pd = perpendicularDistance(mm.get(i), axisOrigin, axisEnd);
            if (pd < 1.0) {
                isInlier[i] = true;
                nInliers++;
            }
            else {
                Logger.info("Auto-Setup:   hole[" + i + "]=("
                        + String.format("%.3f", mm.get(i).getX()) + ", "
                        + String.format("%.3f", mm.get(i).getY())
                        + ") mm excluded — outlier (" + String.format("%.2f", pd)
                        + "mm from the hole line).");
            }
        }
        if (nInliers < 2) {
            throw new Exception("Only " + nInliers + " sprocket hole(s) on the detected line — "
                    + "need at least 2. The pipeline may be picking up non-hole features. "
                    + "Try \"Preview Vision Features\" to see what is being detected.");
        }

        // ---- Build a projection-only list of inliers, sorted along the axis. ----
        final double[] proj = new double[nInliers];
        final int[] inlierIdx = new int[nInliers];
        {
            int idx = 0;
            for (int i = 0; i < n; i++) {
                if (!isInlier[i]) {
                    continue;
                }
                double rx = mm.get(i).getX() - axisOrigin.getX();
                double ry = mm.get(i).getY() - axisOrigin.getY();
                proj[idx] = rx * ux + ry * uy;
                inlierIdx[idx] = i;
                idx++;
            }
        }
        Integer[] order = new Integer[nInliers];
        for (int i = 0; i < nInliers; i++) {
            order[i] = i;
        }
        Arrays.sort(order, Comparator.comparingInt(i -> i));
        // Sort ascending by projection value.
        Arrays.sort(order, Comparator.comparingDouble(i -> proj[i]));

        // Perpendicular (signed) distance from the pick to the hole line.
        // Positive = pick is to the "right" of the axis direction (along +uy,-ux).
        // The sign tells the user which side of the sprocket line the pocket is on.
        double pickRx = pick.getX() - axisOrigin.getX();
        double pickRy = pick.getY() - axisOrigin.getY();
        double pickProj = pickRx * ux + pickRy * uy;
        double signedLateral = pickRx * uy - pickRy * ux;  // cross product
        double lateral = Math.abs(signedLateral);
        String lateralSide = (signedLateral >= 0) ? "right" : "left";

        StringBuilder log = new StringBuilder();
        log.append(String.format("\n    tape axis: angle=%.2f° from hole[%d]→hole[%d] (%.3fmm apart, %d inlier(s))",
                angleTapeAxis, iBestA, iBestB, axisLen, nInliers));
        log.append(String.format("\n    pick projection=%.3fmm, lateral offset=%.3fmm (%s of axis)",
                pickProj, lateral, lateralSide));

        // Suggest the orientation that matches the detected signed lateral.
        int suggestedOrientation = (signedLateral >= 0) ? 0 : 180;
        log.append(String.format("\n    suggested orientation: %d (if the pick ends up on the wrong"
                + " side, flip to %d)", suggestedOrientation,
                (suggestedOrientation == 0) ? 180 : 0));
        for (int k = 0; k < nInliers; k++) {
            int origIdx = inlierIdx[order[k]];
            log.append(String.format("\n    hole[%d] axisPos=%.3fmm at (%.3f, %.3f)",
                    origIdx, proj[order[k]], mm.get(origIdx).getX(), mm.get(origIdx).getY()));
        }

        // --- Find the two consecutive INLIER holes that bracket the pick projection. ---
        int bracketLo = -1, bracketHi = -1;
        for (int k = 0; k < nInliers - 1; k++) {
            double a = proj[order[k]];
            double b = proj[order[k + 1]];
            if (pickProj >= a && pickProj <= b) {
                bracketLo = order[k];
                bracketHi = order[k + 1];
                break;
            }
        }
        if (bracketLo < 0) {
            // Pick projects outside the detected hole span: take the consecutive pair whose
            // midpoint projection is closest to the pick (the end pair nearest the pick).
            double bestMid = Double.MAX_VALUE;
            for (int k = 0; k < nInliers - 1; k++) {
                double mid = (proj[order[k]] + proj[order[k + 1]]) / 2.0;
                double err = Math.abs(mid - pickProj);
                if (err < bestMid) {
                    bestMid = err;
                    bracketLo = order[k];
                    bracketHi = order[k + 1];
                }
            }
            log.append(String.format("\n    pick projects outside the hole span; "
                    + "using nearest end pair hole[%d], hole[%d].",
                    inlierIdx[bracketLo], inlierIdx[bracketHi]));
        }

        // Map from inlier-array indices back to original hole indices.
        int bracketLoOrig = inlierIdx[bracketLo];
        int bracketHiOrig = inlierIdx[bracketHi];

        Logger.info("Auto-Setup: sprocket-hole line analysis:" + log.toString());

        warnIfLateralOutOfRange(lateral);

        Logger.info("Auto-Setup: bracketing pair selected: hole[" + bracketLoOrig + "] at ("
                + String.format("%.3f", mm.get(bracketLoOrig).getX()) + ", "
                + String.format("%.3f", mm.get(bracketLoOrig).getY()) + ") and hole[" + bracketHiOrig
                + "] at (" + String.format("%.3f", mm.get(bracketHiOrig).getX()) + ", "
                + String.format("%.3f", mm.get(bracketHiOrig).getY()) + ").");
        return new Location[]{ mm.get(bracketLoOrig), mm.get(bracketHiOrig) };
    }

    /** Perpendicular distance from point p to the line through a and b (2D, mm). */
    private static double perpendicularDistance(Location p, Location a, Location b) {
        double abx = b.getX() - a.getX();
        double aby = b.getY() - a.getY();
        double abLen = Math.hypot(abx, aby);
        if (abLen < 1e-9) {
            return p.getLinearDistanceTo(a);
        }
        double apx = p.getX() - a.getX();
        double apy = p.getY() - a.getY();
        // |cross product| / |ab|
        return Math.abs(apx * aby - apy * abx) / abLen;
    }

    private void warnIfLateralOutOfRange(double lateral) {
        if (lateral < cassetteMinLateralDistanceMm) {
            Logger.warn("Auto-Setup: pick location is only " + String.format("%.3f", lateral)
                    + "mm from the sprocket-hole line (expected > " + cassetteMinLateralDistanceMm
                    + "mm). The Pick Location may be on a hole rather than on the part pocket.");
        }
        else if (lateral > cassetteMaxLateralDistanceMm) {
            Logger.warn("Auto-Setup: pick location is " + String.format("%.3f", lateral)
                    + "mm from the sprocket-hole line (expected < " + cassetteMaxLateralDistanceMm
                    + "mm). The detected holes may belong to a neighbouring tape.");
        }
    }

    /**
     * Compute the average gap between consecutive holes along the tape axis, using the
     * projections of all detected holes. This is used to derive a scale correction factor
     * that compensates for camera units-per-pixel errors at the current Z.
     */
    private double computeAverageSprocketGap(List<Location> holes, Location hole1, Location hole2) {
        int n = holes.size();
        if (n < 2) {
            return 0;
        }
        // Tape axis direction from the selected pair.
        Location axis = hole1.unitVectorTo(hole2);
        double ux = axis.getX();
        double uy = axis.getY();

        // Project all holes onto this axis, relative to hole1.
        double[] proj = new double[n];
        for (int i = 0; i < n; i++) {
            double rx = holes.get(i).getX() - hole1.getX();
            double ry = holes.get(i).getY() - hole1.getY();
            proj[i] = rx * ux + ry * uy;
        }
        Integer[] order = new Integer[n];
        for (int i = 0; i < n; i++) {
            order[i] = i;
        }
        java.util.Arrays.sort(order, java.util.Comparator.comparingDouble(i -> proj[i]));

        // Sum up consecutive gaps that look like sprocket-lattice steps.
        double sum = 0;
        int count = 0;
        for (int k = 0; k < n - 1; k++) {
            double gap = Math.abs(proj[order[k + 1]] - proj[order[k]]);
            // Skip gaps that are clearly too small (false pocket detection) or too large
            // (non-consecutive holes). Sprocket pitch is 4 mm; allow 2.5–5.5 mm for the
            // stretched measurement.
            if (gap < 2.5 || gap > 5.5) {
                continue;
            }
            sum += gap;
            count++;
        }
        if (count == 0) {
            // Fallback: use the gap of the bracketing pair.
            return hole1.getLinearDistanceTo(hole2);
        }
        return sum / count;
    }

    /**
     * Scale the offset from a reference point to the hole by the given factor.
     * Used to correct for camera units-per-pixel errors. The reference is typically
     * the midpoint of the two bracketing holes so the pick location does not bias
     * the correction.
     */
    private static Location scaleOffsetFromRef(Location hole, Location ref, double factor) {
        Location delta = hole.subtract(ref).derive(null, null, 0.0, null);
        return ref.add(delta.multiply(factor));
    }

    /**
     * Draw the detected features onto the pipeline working image and show it on the camera
     * view. Green = all detected holes, cyan = the selected sprocket-hole pair, yellow
     * crosshair = pick location.
     */
    private void showDetectedFeatures(Camera camera, CvPipeline pipeline, List<Location> holes,
            Location selectedHole1, Location selectedHole2, Location pickLoc, long milliseconds) {
        if (MainFrame.get() == null) {
            return; // not running in the UI
        }
        try {
            Mat mat = pipeline.getWorkingImage().clone();
            // All holes in green.
            if (holes != null) {
                for (Location hole : holes) {
                    org.openpnp.model.Point p = VisionUtils.getLocationPixels(camera, hole);
                    org.opencv.core.Point c = new org.opencv.core.Point(p.x, p.y);
                    Imgproc.circle(mat, c, 6, FluentCv.colorToScalar(Color.green), 1, Imgproc.LINE_AA);
                    Imgproc.circle(mat, c, 1, FluentCv.colorToScalar(Color.green), 2, Imgproc.LINE_AA);
                }
            }
            // Selected pair in cyan.
            if (selectedHole1 != null) {
                org.openpnp.model.Point p = VisionUtils.getLocationPixels(camera, selectedHole1);
                Imgproc.circle(mat, new org.opencv.core.Point(p.x, p.y), 9,
                        FluentCv.colorToScalar(Color.cyan), 2, Imgproc.LINE_AA);
            }
            if (selectedHole2 != null) {
                org.openpnp.model.Point p = VisionUtils.getLocationPixels(camera, selectedHole2);
                Imgproc.circle(mat, new org.opencv.core.Point(p.x, p.y), 9,
                        FluentCv.colorToScalar(Color.cyan), 2, Imgproc.LINE_AA);
            }
            // Pick location as a yellow crosshair.
            if (pickLoc != null && pickLoc.multiply(1, 1, 0, 0).isInitialized()) {
                org.openpnp.model.Point p = VisionUtils.getLocationPixels(camera, pickLoc);
                org.opencv.core.Point c = new org.opencv.core.Point(p.x, p.y);
                int s = 12;
                Imgproc.line(mat, new org.opencv.core.Point(c.x - s, c.y),
                        new org.opencv.core.Point(c.x + s, c.y),
                        FluentCv.colorToScalar(Color.yellow), 2, Imgproc.LINE_AA);
                Imgproc.line(mat, new org.opencv.core.Point(c.x, c.y - s),
                        new org.opencv.core.Point(c.x, c.y + s),
                        FluentCv.colorToScalar(Color.yellow), 2, Imgproc.LINE_AA);
            }
            BufferedImage img = OpenCvUtils.toBufferedImage(mat);
            mat.release();
            MainFrame.get().getCameraViews().getCameraView(camera)
                    .showFilteredImage(img, milliseconds);
        }
        catch (Exception ex) {
            Logger.warn("Could not show features preview: " + ex.getMessage());
        }
    }

    /**
     * Vision-based calibration using the same pairwise sprocket-hole detection as
     * {@link #autoSetup()}.  Unlike the parent's RANSAC line-finding approach, this
     * only needs to find two holes that flank the pick location — far more robust on
     * a cassette baseplate where the camera sees a limited area.
     * <p>
     * On the first call (visionOffset == null), the camera is moved to the pick
     * location, the pipeline detects sprocket holes, and the hole and pick positions
     * are refined.  After that visionOffset is set to a non-null sentinel so we don't
     * re-run on every feed.  If vision fails the geometric positions are kept and a
     * warning is logged.
     */
    @Override
    public void assertCalibrated(boolean tapeFeed) throws Exception {
        Logger.debug("assertCalibrated(tapeFeed=" + tapeFeed + ") visionOffset="
                + (visionOffset == null ? "null" : "set"));
        if (visionOffset != null) {
            return; // already calibrated
        }
        if (getHole1Location().convertToUnits(LengthUnit.Millimeters)
                .getLinearDistanceTo(getHole2Location()) < 3) {
            throw new Exception("Feeder " + getName()
                    + " sprocket hole locations undefined/too close together.");
        }
        try {
            calibratePickFromVision();
        }
        catch (Exception e) {
            Logger.warn(e, "Vision calibration failed for feeder " + getName()
                    + ": {}. Using geometric positions.", e.getMessage());
            // Set visionOffset to a non-null sentinel so we fall back to geometric
            // positions and don't retry every feed.
            visionOffset = Location.origin;
        }
    }

    /**
     * Move the camera to the current pick location, run the vision pipeline to detect
     * the two sprocket holes flanking the pocket, and refine hole1, hole2 and the pick
     * location from the detected hole positions.
     */
    private void calibratePickFromVision() throws Exception {
        Camera camera = getCamera();
        Location pickLoc = getLocation().convertToUnits(LengthUnit.Millimeters);
        if (!pickLoc.multiply(1, 1, 0, 0).isInitialized()) {
            throw new Exception("Pick Location is not set.");
        }

        ensureCameraZ(camera, true);
        MovableUtils.moveToLocationAtSafeZ(camera, pickLoc.derive(null, null, null, 0.0));

        // Use autoSetup-style pipeline (full-camera search range, no OCR).
        try (CvPipeline pipeline = getCvPipeline(camera, true, false, true)) {
            pipeline.process();

            List<CvStage.Result.Circle> circles = extractCirclesFromPipeline(pipeline);
            List<Location> holes = filterHolesByDiameter(camera, circles);
            Logger.info("Feed-time calibration: " + circles.size() + " circle(s), "
                    + holes.size() + " passed the ~" + sprocketHoleDiameterMm
                    + "mm diameter filter.");

            Location[] pair = findBestSprocketHolePair(holes, pickLoc);
            Location hole1 = pair[0];
            Location hole2 = pair[1];

            // Order holes consistently with the feeder orientation.
            double expectedDx = getOffsetXWithOrientation(0, 4, orientation);
            double expectedDy = getOffsetYWithOrientation(0, 4, orientation);
            double actualDx = hole1.getX() - hole2.getX();
            double actualDy = hole1.getY() - hole2.getY();
            if (actualDx * expectedDx + actualDy * expectedDy < 0) {
                Location swap = hole2;
                hole2 = hole1;
                hole1 = swap;
            }

            // Derive the pick location from detected hole1 using EIA-481 geometry.
            double holeToPickDx = getOffsetXWithOrientation(-sprocketHoleToCavityMm, 2, orientation)
                                - getOffsetXWithOrientation(0, 0, orientation);
            double holeToPickDy = getOffsetYWithOrientation(-sprocketHoleToCavityMm, 2, orientation)
                                - getOffsetYWithOrientation(0, 0, orientation);
            Location derivedPick = hole1.subtract(
                    new Location(LengthUnit.Millimeters, holeToPickDx, holeToPickDy, 0, 0));

            double pickError = derivedPick.getLinearDistanceTo(pickLoc);
            if (pickError > 2.0) {
                Logger.warn("Feed-time calibration: derived pick differs from stored pick by "
                        + String.format("%.2f", pickError) + "mm. Using derived pick.");
            }
            else {
                Logger.info("Feed-time calibration: derived pick matches stored pick ("
                        + String.format("%.2f", pickError) + "mm difference).");
            }

            // Update positions. Use the stored pick Z / rotation.
            Length pickZ = pickLoc.getLengthZ();
            hole1 = hole1.deriveLengths(null, null, pickZ, null);
            hole2 = hole2.deriveLengths(null, null, pickZ, null);
            Location updatedPick = derivedPick.derive(null, null, null, pickLoc.getRotation());
            updatedPick = updatedPick.deriveLengths(null, null, pickZ, null);

            double measuredPitchMm = hole1.getLinearDistanceTo(hole2);
            Logger.info("Feed-time calibration: hole1=("
                    + String.format("%.3f", hole1.getX()) + ", "
                    + String.format("%.3f", hole1.getY()) + ") hole2=("
                    + String.format("%.3f", hole2.getX()) + ", "
                    + String.format("%.3f", hole2.getY()) + ") pitch="
                    + String.format("%.3f", measuredPitchMm) + "mm");

            setHole1Location(hole1);
            setHole2Location(hole2);
            setLocation(updatedPick);

            // Mark calibration as done.
            visionOffset = Location.origin;
        }
    }

    /**
     * Cassette feeders identify their parts through the baseplate firmware (the part id
     * is part of the discovery response), so OCR-based part identification is not needed.
     * Hiding the OCR controls also prevents a stale OCR region from triggering the
     * AffineWarp stage of the vision pipeline.
     */
    @Override
    public boolean isOcrSupported() {
        return false;
    }

    /**
     * Disable OCR-on-job-start by default for cassette feeders.
     * OCR-triggered vision calibration fails on baseplate hole patterns.
     */
    @Override
    public boolean isOcrDiscoverOnJobStart() {
        return false;
    }

    @Override
    public void prepareForJob(boolean visit) throws Exception {
        // For CassetteFeeder, skip vision-based job preparation.
        // Only verify hole distance sanity check via assertCalibrated.
        if (visit && getVisionOffset() == null) {
            assertCalibrated(false);
        }
    }

    @Override
    public void feed(Nozzle nozzle) throws Exception {
        Logger.debug("feed()");
        Actuator actuator = configureActuator();
        Head head = nozzle.getHead();
        if (actuator == null) {
            throw new Exception(String.format("No feed actuator assigned to feeder %s",
                    getName()));
        }

        if (getFeedCount() % getPartsPerFeedCycle() == 0) {
            // Modulo of feed count is zero - no more parts there to pick, must feed 
            // Make sure we're calibrated
            if(getSubType()!=1){
                assertCalibrated(false);
                long feedsPerPart = (long)Math.ceil(getPartPitch().divide(getFeedPitch()));
                long n = getFeedMultiplier()*feedsPerPart;
                for (long i = 0; i < n; i++) {  // perform multiple feed actuations if required
                    actuator.read(String.format("R%d C%d TC%d TR%d AD1;",row,col,totalCol,totalRow));
                }
            }else{
                actuator.read(String.format("R%d C%d TC%d TR%d AD1;",row,col,totalCol,totalRow));
            }
        }else{
            Logger.debug("Multi parts feed: skipping tape feed at feed count " + getFeedCount());
        }
        
        if(getSubType()!=1){
            // Make sure we're calibrated after type feed
            assertCalibrated(true);
        }
        // increment feed count 
        setFeedCount(getFeedCount()+1);        
    }

    @Override
    public void postPick(Nozzle nozzle) throws Exception {
        Actuator actuator = configureActuator();
        // post pick action, for loosepart feeder, turn off the light
        actuator.read(String.format("R%d C%d TC%d TR%d AD2;",row,col,totalCol,totalRow));
    }

    /**
     * Column direction is perpendicular to the rail direction, oriented so that
     * positive column index yields positive X (vertical layout) or positive Y
     * (horizontal layout), matching legacy isVerticalLayout behavior.
     */
    private double getColDirectionX() {
        return isVerticalLayout ? rowDirectionY : -rowDirectionY;
    }

    private double getColDirectionY() {
        return isVerticalLayout ? -rowDirectionX : rowDirectionX;
    }

    private Location calculateHole1LocationFromRowColHeight(int oRow, int oCol, double ox, double oy,double height) {
        double rs = rowSpacing.getValue();
        int row = oRow;
        int col = oCol;
        double colDX = getColDirectionX(), colDY = getColDirectionY();
        double skip = 0;
        if (continuousRowCol > 0) {
            skip = (isVerticalLayout ? (col/continuousRowCol) : (row/continuousRowCol)) * skippedRowCol * rs;
        }
        double rowX = row * rs * rowDirectionX;
        double rowY = row * rs * rowDirectionY;
        double colX = col * rs * colDX + (isVerticalLayout ? skip * colDX : skip * rowDirectionX);
        double colY = col * rs * colDY + (isVerticalLayout ? skip * colDY : skip * rowDirectionY);
        // hole1 sits -sprocketHoleToCavityMm behind the pick
        return new Location(Configuration.get().getSystemUnits(), 
            baseplateOffsetX.getValue() + rowX + colX + getOffsetXWithOrientation(ox - sprocketHoleToCavityMm, oy+2, orientation), 
            baseplateOffsetY.getValue() + rowY + colY + getOffsetYWithOrientation(ox - sprocketHoleToCavityMm, oy+2, orientation), 
            baseplateOffsetZ.add(new Length(height,LengthUnit.Millimeters)).getValue(), 0);
    }
    private Location calculateHole2LocationFromRowColHeight(int oRow, int oCol, double ox, double oy,double height) {
        double rs = rowSpacing.getValue();
        int row = oRow;
        int col = oCol;
        double colDX = getColDirectionX(), colDY = getColDirectionY();
        double skip = 0;
        if (continuousRowCol > 0) {
            skip = (isVerticalLayout ? (col/continuousRowCol) : (row/continuousRowCol)) * skippedRowCol * rs;
        }
        double rowX = row * rs * rowDirectionX;
        double rowY = row * rs * rowDirectionY;
        double colX = col * rs * colDX + (isVerticalLayout ? skip * colDX : skip * rowDirectionX);
        double colY = col * rs * colDY + (isVerticalLayout ? skip * colDY : skip * rowDirectionY);
        // hole2 sits -sprocketHoleToCavityMm behind the pick
        return new Location(Configuration.get().getSystemUnits(), 
            baseplateOffsetX.getValue() + rowX + colX + getOffsetXWithOrientation(ox - sprocketHoleToCavityMm, oy-2, orientation), 
            baseplateOffsetY.getValue() + rowY + colY + getOffsetYWithOrientation(ox - sprocketHoleToCavityMm, oy-2, orientation), 
            baseplateOffsetZ.add(new Length(height,LengthUnit.Millimeters)).getValue(), 0);
    }    

    private Location calculatePickLocationFromRowColHeight(int oRow, int oCol, double ox, double oy,double height) {
        double rs = rowSpacing.getValue();
        int row = oRow;
        int col = oCol;
        double colDX = getColDirectionX(), colDY = getColDirectionY();
        double skip = 0;
        if (continuousRowCol > 0) {
            skip = (isVerticalLayout ? (col/continuousRowCol) : (row/continuousRowCol)) * skippedRowCol * rs;
        }
        double rowX = row * rs * rowDirectionX;
        double rowY = row * rs * rowDirectionY;
        double colX = col * rs * colDX + (isVerticalLayout ? skip * colDX : skip * rowDirectionX);
        double colY = col * rs * colDY + (isVerticalLayout ? skip * colDY : skip * rowDirectionY);
        return new Location(Configuration.get().getSystemUnits(), 
            baseplateOffsetX.add(new Length(rowX + colX + getOffsetXWithOrientation(ox, oy, orientation),
                LengthUnit.Millimeters)).getValue(),
            baseplateOffsetY.add(new Length(rowY + colY + getOffsetYWithOrientation(ox, oy, orientation),
                LengthUnit.Millimeters)).getValue(),
            baseplateOffsetZ.add(new Length(height,LengthUnit.Millimeters)).getValue(), 0);
    }

    private double getOffsetXWithOrientation(double ox, double oy, int ori){
        switch (ori) {
            case 0:
                return ox;
            case 90:
                return oy;
            case 180:
                return -ox;
            case 270:
                return -oy;
            default:
                return ox;
        }
    }
    private double getOffsetYWithOrientation(double ox, double oy, int ori){
        switch (ori) {
            case 0:
                return oy;
            case 90:
                return -ox;
            case 180:
                return -oy;
            case 270:
                return ox;
            default:
                return oy;
        }
    }    
}
