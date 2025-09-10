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
import org.openpnp.machine.reference.feeder.wizards.ReferenceLoosePartFeederConfigurationWizard;
import org.openpnp.model.Configuration;
import org.openpnp.model.Length;
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
        
        actuator.read(String.format("R:%d,C:%d,TC:%d,TR:%d,AD:1;",row,col,totalCol,totalRow));
        super.feed(nozzle);
    }

    
    @Override
    public void postPick(Nozzle nozzle) throws Exception {
        Actuator actuator = configureActuator();
        // post pick action, for loosepart feeder, turn off the light
        actuator.read(String.format("R:%d,C:%d,TC:%d,TR:%d,AD:2;",row,col,totalCol,totalRow));
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
