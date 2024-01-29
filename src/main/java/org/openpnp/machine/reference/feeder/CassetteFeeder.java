package org.openpnp.machine.reference.feeder;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import org.openpnp.ConfigurationListener;
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
import org.openpnp.spi.Machine;
import org.openpnp.spi.Nozzle;
import org.pmw.tinylog.Logger;
import org.simpleframework.xml.Element;
import org.openpnp.spi.Driver;
import org.openpnp.spi.Feeder;
import org.openpnp.spi.Head;

public class CassetteFeeder extends ReferencePushPullFeeder {

    public static final String ACTUATOR_DISCOVER_NAME = "CassetteFeederDiscovery";
    @Element(required = false)
    protected Length baseplateOffsetX= new Length(0,LengthUnit.Millimeters);
    @Element(required = false)
    protected Length baseplateOffsetY= new Length(0,LengthUnit.Millimeters);
    @Element(required = false)
    protected Length baseplateOffsetZ= new Length(0,LengthUnit.Millimeters);
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
            }
        });
        //use circular symmetry for pipeline
        resetPipeline(PipelineType.CircularSymmetry);
        
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
                    gcodeDriver.setCommand(actuator, GcodeDriver.CommandType.ACTUATE_STRING_COMMAND, "M888 {StringValue};");
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
            String response = actuator.read(String.format("R:%d,C:%d,TC:%d,TR:%d,PI:%d,N:%s;",row,col,totalCol,totalRow,
                (int)(getPartPitch().getValue()*10),getPart()==null?getName():getPart().getId()));
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
            String response = actuator.read(String.format("TR:%d,TC:%d;",totalRow,totalCol));
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
            if(!(feeder instanceof CassetteFeeder)){
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

                CassetteFeeder theFeeder = (CassetteFeeder) feeder;
 
                updateFeeder(feederInfo, theFeeder);
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
            Logger.debug("Adding new feeders {}",key);
            Map<String,String> feederInfo = feeders.get(key);
            if(!(feederInfo.containsKey("c") && feederInfo.containsKey("r")&& feederInfo.containsKey("h"))){
                Logger.warn("Feeder id:{} lack of row,column or height info, skipping..");
                continue;
            }

            CassetteFeeder newFeeder = new CassetteFeeder();
            newFeeder.id = key;
            updateFeeder(feederInfo, newFeeder);
            
            try {
                Logger.info("Added feeder id:{} name:{} at col:{} row:{}",key,feederInfo.get("n"),newFeeder.col,newFeeder.row);
                Configuration.get().getMachine().addFeeder(newFeeder);
            } catch (Exception e) {                
                Logger.error("now able to add feeder with id:{}",key);
                e.printStackTrace();
            }
        }
    }

    private void updateFeeder(Map<String, String> feederInfo, CassetteFeeder theFeeder) {

        int col = Integer.parseInt(feederInfo.get("c"));
        int row = Integer.parseInt(feederInfo.get("r"));
        double ox = feederInfo.containsKey("ox")?Double.parseDouble(feederInfo.get("ox")):0;
        double oy = feederInfo.containsKey("oy")?Double.parseDouble(feederInfo.get("oy")):0;
        double h = Double.parseDouble(feederInfo.get("h"));
        double pi = feederInfo.containsKey("pi")?Double.parseDouble(feederInfo.get("pi")):getPartPitch().getValue()*10;

        theFeeder.setCol(col);
        theFeeder.setRow(row);
        theFeeder.setTotalCol(totalCol);
        theFeeder.setTotalRow(totalRow);
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
        //generate dummy ocr region
        theFeeder.setOcrRegion(new RegionOfInterest(
            calculateHole1LocationFromRowColHeight(row,col,ox,oy,h),
            calculateHole1LocationFromRowColHeight(row,col,ox,oy,h).derive(2.0,0.0, 0.0,0.0),
            calculateHole1LocationFromRowColHeight(row,col,ox,oy,h).derive(0.0,-2.0, 0.0,0.0),false
        ));
        theFeeder.setEnabled(true);
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
            assertCalibrated(false);
            long feedsPerPart = (long)Math.ceil(getPartPitch().divide(getFeedPitch()));
            long n = getFeedMultiplier()*feedsPerPart;
            for (long i = 0; i < n; i++) {  // perform multiple feed actuations if required
                actuator.read(String.format("R:%d,C:%d,TC:%d,TR:%d,AD:1;",row,col,totalCol,totalRow));
            }
        }else{
            Logger.debug("Multi parts feed: skipping tape feed at feed count " + getFeedCount());
        }
        
        // Make sure we're calibrated after type feed
        assertCalibrated(true);
        // increment feed count 
        setFeedCount(getFeedCount()+1);        
    }



    private Location calculateHole1LocationFromRowColHeight(int row, int col, double ox, double oy,double height) {
        
        return new Location(Configuration.get().getSystemUnits(), 
            col*8+baseplateOffsetX.getValue()+ getOffsetXWithOrientation(ox+4, oy+2, orientation) + 
                ((isVerticalLayout&&continuousRowCol>0)?(col/continuousRowCol)*skippedRowCol*8:0), 
            row*8+baseplateOffsetY.getValue()+ getOffsetYWithOrientation(ox+4, oy+2, orientation) + 
                ((!isVerticalLayout&&continuousRowCol>0)?(row/continuousRowCol)*skippedRowCol*8:0), 
                baseplateOffsetZ.add(new Length(height,LengthUnit.Millimeters)).getValue(), 0);
    }
    private Location calculateHole2LocationFromRowColHeight(int row, int col, double ox, double oy,double height) {
        
        return new Location(Configuration.get().getSystemUnits(), 
            col*8+baseplateOffsetX.getValue()+ getOffsetXWithOrientation(ox+4, oy-2, orientation) + 
                ((isVerticalLayout&&continuousRowCol>0)?(col/continuousRowCol)*skippedRowCol*8:0), 
            row*8+baseplateOffsetY.getValue()+ getOffsetYWithOrientation(ox+4, oy-2, orientation) + 
                ((!isVerticalLayout&&continuousRowCol>0)?(row/continuousRowCol)*skippedRowCol*8:0), 
                baseplateOffsetZ.add(new Length(height,LengthUnit.Millimeters)).getValue(), 0);
    }    

    private Location calculatePickLocationFromRowColHeight(int row, int col, double ox, double oy,double height) {
        
        return new Location(Configuration.get().getSystemUnits(), 
            baseplateOffsetX.add(new Length(col*8+getOffsetXWithOrientation(ox, oy, orientation)+
                ((isVerticalLayout&&continuousRowCol>0)?(col/continuousRowCol)*skippedRowCol*8:0),LengthUnit.Millimeters)).getValue(),
            baseplateOffsetY.add(new Length(row*8+getOffsetYWithOrientation(ox, oy, orientation) + 
                ((!isVerticalLayout&&continuousRowCol>0)?(row/continuousRowCol)*skippedRowCol*8:0),LengthUnit.Millimeters)).getValue(), 
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
