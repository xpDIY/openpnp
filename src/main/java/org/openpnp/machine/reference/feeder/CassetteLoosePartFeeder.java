/*
 * Copyright (C) 2011 Jason von Nieda <jason@vonnieda.org>
 * 
 * This file is part of OpenPnP.
 * 
 * OpenPnP is free software: you can redistribute it and/or modify it under the terms of the GNU
 * General Public License as published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 * 
 * OpenPnP is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along with OpenPnP. If not, see
 * <http://www.gnu.org/licenses/>.
 * 
 * For more information about OpenPnP visit http://openpnp.org
 */

package org.openpnp.machine.reference.feeder;

import java.util.List;

import javax.swing.Action;

import org.apache.commons.io.IOUtils;
import org.opencv.core.RotatedRect;
import org.openpnp.gui.MainFrame;
import org.openpnp.gui.support.PropertySheetWizardAdapter;
import org.openpnp.gui.support.Wizard;
import org.openpnp.machine.reference.ReferenceActuator;
import org.openpnp.machine.reference.ReferenceFeeder;
import org.openpnp.machine.reference.driver.GcodeDriver;
import org.openpnp.machine.reference.feeder.wizards.CassetteLoosePartFeederConfigurationWizard;
import org.openpnp.model.Configuration;
import org.openpnp.model.Length;
import org.openpnp.model.LengthUnit;
import org.openpnp.model.Location;
import org.openpnp.spi.Actuator;
import org.openpnp.spi.Camera;
import org.openpnp.spi.Driver;
import org.openpnp.spi.Head;
import org.openpnp.spi.Machine;
import org.openpnp.spi.Nozzle;
import org.openpnp.spi.PropertySheetHolder;
import org.openpnp.util.MovableUtils;
import org.openpnp.util.OpenCvUtils;
import org.openpnp.util.VisionUtils;
import org.openpnp.vision.pipeline.CvPipeline;
import org.pmw.tinylog.Logger;
import org.simpleframework.xml.Element;

public class CassetteLoosePartFeeder extends ReferenceLoosePartFeeder {

    public static final String ACTUATOR_DISCOVER_NAME = "CassetteFeederDiscovery";

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
    
    // Working plane height - used as reference for height calculations
    @Element(required = false)
    protected Length baseplateOffsetZ = new Length(0, LengthUnit.Millimeters);
    
    // Track if settings have changed and need to be saved to feeder
    private transient boolean needsSave = false;

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

    public Length getBaseplateOffsetZ() {
        return baseplateOffsetZ;
    }
    
    public void setBaseplateOffsetZ(Length offset) {
        Object oldValue = this.baseplateOffsetZ;
        this.baseplateOffsetZ = offset;
        firePropertyChange("baseplateOffsetZ", oldValue, offset);
    }

    @Override
    public void setLocation(Location location) {
        // Track if Z height changed - save immediately to feeder flash
        if (this.location != null && location != null) {
            double oldZ = this.location.convertToUnits(LengthUnit.Millimeters).getZ();
            double newZ = location.convertToUnits(LengthUnit.Millimeters).getZ();
            
            // If Z changed by more than 0.01mm, save immediately
            if (Math.abs(oldZ - newZ) > 0.01) {
                needsSave = true;
                Logger.debug("Location Z changed for feeder {}, saving to flash", getName());
                // Update location first, then save in machine context
                super.setLocation(location);
                // Save immediately in a machine task context
                try {
                    Configuration.get().getMachine().execute(() -> {
                        saveToFeeder();
                        return null;
                    });
                } catch (Exception e) {
                    Logger.error(e, "Failed to auto-save feeder configuration after location change");
                }
                return;
            }
        }
        super.setLocation(location);
    }

    /**
     * Save the feeder configuration (including height) to feeder flash.
     * Must be called from within a machine task context.
     * Height is stored in 0.1mm units (integer), so we:
     * 1. Subtract the working plane height (baseplateOffsetZ)
     * 2. Multiply by 10 to convert mm to 0.1mm units
     * 3. Take integer value
     */
    public void saveToFeeder() {
        Logger.debug("Entering saveToFeeder for loose part feeder");
        Actuator actuator = configureActuator();
        try {
            if (actuator == null) {
                throw new Exception("No actuator configured for feeder " + getName());
            }
            
            // Convert current Z location to height value for feeder storage
            double zInMm = location.convertToUnits(LengthUnit.Millimeters).getZ();
            double heightRelativeToWorkingPlane = zInMm - baseplateOffsetZ.convertToUnits(LengthUnit.Millimeters).getValue();
            int heightForFeeder = (int) Math.round(heightRelativeToWorkingPlane * 10.0);
            
            // Send command to save configuration to feeder
            String response = actuator.read(String.format("R%d C%d TC%d TR%d H%d ST%d N%s;", 
                row, col, totalCol, totalRow, heightForFeeder, subType, 
                getPart() == null ? getName() : getPart().getId()));
            Logger.info("Saved feeder config with height {} ({}*0.1mm) to R{} C{}, response: {}", 
                heightForFeeder, heightForFeeder, row, col, response);
            needsSave = false;  // Clear the save flag
        } catch (Exception e) {
            Logger.error(e, "Failed to save feeder configuration");
        }
    }

    
    public boolean getIsVerticalLayout(){
        return isVerticalLayout;
    }

    public void setIsVerticalLayout(boolean val){
        Object oldValue = this.isVerticalLayout;
        this.isVerticalLayout = val;
        firePropertyChange("isVerticalLayout", oldValue, val);
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
        
        // Auto-save if settings changed (e.g., user modified Z height and clicked Apply)
        if (needsSave) {
            Logger.info("Auto-saving feeder {} configuration to flash before feeding", getName());
            saveToFeeder();
        }
        
        // Turn off top lighting and prevent camera from turning it back on during vision
        Camera camera = head.getDefaultCamera();
        Actuator lightActuator = camera.getLightActuator();
        boolean originalBeforeCaptureLightOn = false;
        
        if (lightActuator != null) {
            // Save original setting and disable automatic lighting
            if (camera instanceof org.openpnp.spi.base.AbstractCamera) {
                org.openpnp.spi.base.AbstractCamera abstractCamera = (org.openpnp.spi.base.AbstractCamera) camera;
                originalBeforeCaptureLightOn = abstractCamera.isBeforeCaptureLightOn();
                abstractCamera.setBeforeCaptureLightOn(false);
            }
            // Turn off the light
            lightActuator.actuate(false);
        }
        
        actuator.read(String.format("R%d C%d TC%d TR%d AD1;",row,col,totalCol,totalRow));
        
        try {
            super.feed(nozzle);
        } finally {
            // Restore original camera light setting
            if (lightActuator != null && camera instanceof org.openpnp.spi.base.AbstractCamera) {
                org.openpnp.spi.base.AbstractCamera abstractCamera = (org.openpnp.spi.base.AbstractCamera) camera;
                abstractCamera.setBeforeCaptureLightOn(originalBeforeCaptureLightOn);
            }
        }
    }

    
    @Override
    public void postPick(Nozzle nozzle) throws Exception {
        Actuator actuator = configureActuator();
        // post pick action, for loosepart feeder, turn off the light
        actuator.read(String.format("R%d C%d TC%d TR%d AD2;",row,col,totalCol,totalRow));
    }

    @Override
    public Wizard getConfigurationWizard() {
        return new CassetteLoosePartFeederConfigurationWizard(this);
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


}
