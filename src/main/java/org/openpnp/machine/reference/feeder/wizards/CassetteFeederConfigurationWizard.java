package org.openpnp.machine.reference.feeder.wizards;

import javax.swing.AbstractAction;
import javax.swing.Action;
import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextField;
import javax.swing.SwingUtilities;
import javax.swing.border.TitledBorder;

import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;

import org.jdesktop.beansbinding.AutoBinding.UpdateStrategy;
import org.openpnp.Translations;
import org.openpnp.events.FeederSelectedEvent;
import org.openpnp.gui.components.ComponentDecorators;
import org.openpnp.gui.support.AbstractConfigurationWizard;
import org.openpnp.gui.support.DoubleConverter;
import org.openpnp.gui.support.IntegerConverter;
import org.openpnp.gui.support.LengthConverter;
import org.openpnp.gui.support.LongConverter;
import org.openpnp.gui.support.MutableLocationProxy;
import org.openpnp.machine.reference.feeder.CassetteFeeder;
import org.openpnp.machine.reference.feeder.ReferencePushPullFeeder;
import org.openpnp.model.Configuration;
import org.openpnp.spi.Camera;
import org.openpnp.util.MovableUtils;
import org.openpnp.util.UiUtils;
import org.pmw.tinylog.Logger;


@SuppressWarnings("serial")
public class CassetteFeederConfigurationWizard extends AbstractConfigurationWizard {
    private final CassetteFeeder feeder;
    private JPanel panelLocations;
    private JPanel panelParam;
    private JPanel panelLayout;
    private JButton btnDiscover;
    private JButton btnSave;
    private JLabel labelOffsetX;
    private JLabel labelOffsetY;
    private JLabel labelOffsetZ;
    private JTextField textFieldBaseplateOffsetX;
    private JTextField textFieldBaseplateOffsetY;
    private JTextField textFieldBaseplateOffsetZ;
    private JLabel labelContinuousRowCol;
    private JLabel labelSkippedRowCol;
    private JTextField textFieldContinuousRowCol;
    private JTextField textFieldSkippedRowCol;
    private JLabel labelIsLayoutVertical;
    private JCheckBox checkBoxIsVertical;
    private JLabel labelOrientation;
    private JTextField textFieldOrientation;

    private JLabel labelTotalRow;
    private JLabel labelTotalCol;
    private JTextField textFieldTotalRow;
    private JTextField textFieldTotalCol;

    protected Action baseApplyAction;

    public CassetteFeederConfigurationWizard(CassetteFeeder feeder){
        //super(feeder, false);
        super();
        this.feeder = feeder;

        JPanel panelFields = new JPanel();
        panelFields.setLayout(new BoxLayout(panelFields, BoxLayout.Y_AXIS));

        panelLocations = new JPanel();
        panelLocations.setBorder(new TitledBorder(null, "Discover", TitledBorder.LEADING,
                TitledBorder.TOP, null, null));
        panelFields.add(panelLocations);                

        btnDiscover = new JButton(discoverFeederAction);
        btnDiscover.setToolTipText("Automatically discover feeders that has been plug into the base plate.");
        btnDiscover.setText("Discover Feeders");
        panelLocations.add(btnDiscover, "1, 1, default, fill");      
        
        btnSave = new JButton(saveFeederAction);
        btnSave.setToolTipText("Save the current settings to feeder.");
        btnSave.setText("Save to Feeder");
        panelLocations.add(btnSave, "1, 1, default, fill");      

        //For layout editing
        panelLayout = new JPanel();
        panelLayout.setBorder(new TitledBorder(null, "Baseplate Layout", TitledBorder.LEADING,
                TitledBorder.TOP, null, null));
        panelLayout.setLayout(new GridLayout(0,6));

        labelIsLayoutVertical = new JLabel("Vertical layout (Col mode)");
        panelLayout.add(labelIsLayoutVertical);
        checkBoxIsVertical = new JCheckBox();
        panelLayout.add(checkBoxIsVertical);

        labelContinuousRowCol =new JLabel("Continuous Row/Col");
        panelLayout.add(labelContinuousRowCol);
        textFieldContinuousRowCol = new JTextField();
        panelLayout.add(textFieldContinuousRowCol);
        labelSkippedRowCol =new JLabel("Skipped Row/Col");
        panelLayout.add(labelSkippedRowCol);
        textFieldSkippedRowCol = new JTextField();
        panelLayout.add(textFieldSkippedRowCol);

        labelOffsetX = new JLabel("Offset X");
        panelLayout.add(labelOffsetX,"1,1");

        textFieldBaseplateOffsetX = new JTextField();
        panelLayout.add(textFieldBaseplateOffsetX,"1,1");

        labelOffsetY = new JLabel("Offset Y");
        panelLayout.add(labelOffsetY,"1,1");

        textFieldBaseplateOffsetY = new JTextField();
        panelLayout.add(textFieldBaseplateOffsetY,"1,1");

        labelOffsetZ = new JLabel("Offset Z");
        panelLayout.add(labelOffsetZ,"1,1");

        textFieldBaseplateOffsetZ = new JTextField();
        panelLayout.add(textFieldBaseplateOffsetZ,"1,1");        

        labelTotalRow = new JLabel("Total Row");
        panelLayout.add(labelTotalRow,"1,1");

        textFieldTotalRow = new JTextField();
        panelLayout.add(textFieldTotalRow,"1,1");

        labelTotalCol = new JLabel("Total Column");
        panelLayout.add(labelTotalCol,"1,1");

        textFieldTotalCol = new JTextField();
        panelLayout.add(textFieldTotalCol,"1,1");
        panelFields.add(panelLayout);        

        // parameter editing
        panelParam = new JPanel();
        panelParam.setBorder(new TitledBorder(null, "Feeder Parameters", TitledBorder.LEADING,
                TitledBorder.TOP, null, null));
        panelParam.setLayout(new GridLayout(0,8));
        labelOrientation = new JLabel("Orientation (deg)");
        panelParam.add(labelOrientation);
        textFieldOrientation = new JTextField();
        textFieldOrientation.setToolTipText("Sets the orientation of the feeder in degree. Can only be one of 0,90,180,270");
        panelParam.add(textFieldOrientation);
        

        panelFields.add(panelParam);        
        contentPanel.add(panelFields);

        baseApplyAction = applyAction;
    }

    @Override
    protected void saveToModel() {
        Logger.debug("Saving model");
        
        super.saveToModel();
    }

        private Action discoverFeederAction =
            new AbstractAction("Discovery") {
        {
            putValue(Action.SHORT_DESCRIPTION,
                    "<html>Automatically discover feeders and setup the position.</html>");
        }

        @Override
        public void actionPerformed(ActionEvent e) {
            UiUtils.messageBoxOnException(() -> {
                Logger.debug("Performing discovery of feeders");
                UiUtils.submitUiMachineTask(feeder::discoverFeeders);
                wizardContainer.wizardCompleted(null);
            });
        }
    };
        private Action saveFeederAction =
        new AbstractAction("SaveFeder") {
        {
        putValue(Action.SHORT_DESCRIPTION,
                "<html>Save current settings to feeder.</html>");
        }

        @Override
        public void actionPerformed(ActionEvent e) {
        UiUtils.messageBoxOnException(() -> {
                Logger.debug("saving settings to feeder");
                UiUtils.submitUiMachineTask(feeder::saveToFeeder);
                wizardContainer.wizardCompleted(null);
        });
        }
        };


        @Override
        public void createBindings() {
            // TODO Auto-generated method stub
                    //super.createBindings();
        LengthConverter lengthConverter = new LengthConverter();
        IntegerConverter intConverter = new IntegerConverter();
        LongConverter longConverter = new LongConverter();
        DoubleConverter doubleConverter =
                new DoubleConverter(Configuration.get().getLengthDisplayFormat());

        addWrappedBinding(feeder, "baseplateOffsetX", textFieldBaseplateOffsetX, "text",
                lengthConverter);
        addWrappedBinding(feeder, "baseplateOffsetY", textFieldBaseplateOffsetY, "text",
                lengthConverter);
        addWrappedBinding(feeder, "baseplateOffsetZ", textFieldBaseplateOffsetZ, "text",
                lengthConverter);
        addWrappedBinding(feeder, "continuousRowCol", textFieldContinuousRowCol,"text", intConverter);
        addWrappedBinding(feeder, "skippedRowCol", textFieldSkippedRowCol,"text", intConverter);
        addWrappedBinding(feeder, "isVerticalLayout", checkBoxIsVertical,"selected");
        addWrappedBinding(feeder, "orientation", textFieldOrientation,"text", intConverter);
        addWrappedBinding(feeder, "totalRow", textFieldTotalRow,"text", intConverter);
        addWrappedBinding(feeder, "totalCol", textFieldTotalCol,"text", intConverter);

        ComponentDecorators.decorateWithAutoSelectAndLengthConversion(textFieldBaseplateOffsetX);
        ComponentDecorators.decorateWithAutoSelectAndLengthConversion(textFieldBaseplateOffsetY);

        
            
        }
}
