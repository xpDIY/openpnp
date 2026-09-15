package org.openpnp.machine.reference.feeder.wizards;

import java.awt.event.ActionEvent;

import javax.swing.AbstractAction;
import javax.swing.Action;
import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextField;
import javax.swing.border.TitledBorder;

import org.jdesktop.beansbinding.AutoBinding.UpdateStrategy;
import org.openpnp.gui.components.ComponentDecorators;
import org.openpnp.gui.components.LocationButtonsPanel;
import org.openpnp.gui.support.AbstractConfigurationWizard;
import org.openpnp.gui.support.DoubleConverter;
import org.openpnp.gui.support.IntegerConverter;
import org.openpnp.gui.support.LengthConverter;
import org.openpnp.gui.support.MutableLocationProxy;
import org.openpnp.machine.reference.feeder.CassetteFeederConfigurator;
import org.openpnp.model.Configuration;
import org.openpnp.model.Location;
import org.openpnp.util.UiUtils;

import com.jgoodies.forms.layout.ColumnSpec;
import com.jgoodies.forms.layout.FormLayout;
import com.jgoodies.forms.layout.FormSpecs;
import com.jgoodies.forms.layout.RowSpec;

/**
 * Wizard for {@link CassetteFeederConfigurator}. Edits the shared baseplate
 * layout (the single source of truth for all {@code CassetteAutoFeeder}s) and
 * triggers firmware feeder discovery.
 */
@SuppressWarnings("serial")
public class CassetteFeederConfiguratorConfigurationWizard extends AbstractConfigurationWizard {
    private final CassetteFeederConfigurator feeder;

    private JTextField textFieldBaseplateOffsetX, textFieldBaseplateOffsetY, textFieldBaseplateOffsetZ;
    private JTextField textFieldRowSpacing, textFieldRowDirectionX, textFieldRowDirectionY;
    private JLabel labelTotalRow;
    private JTextField textFieldTotalCol;
    private JCheckBox checkBoxIsVerticalLayout;
    private JTextField textFieldContinuousRowCol, textFieldSkippedRowCol;
    private JTextField textFieldOrientation;

    // Stage-1 calibration from the 2nd and 2nd-last slot centers + slot count
    private JTextField textFieldCalSecondX, textFieldCalSecondY, textFieldCalSecondZ;
    private JTextField textFieldCalSecondLastX, textFieldCalSecondLastY, textFieldCalSecondLastZ;
    private JTextField textFieldCalTotalSlots;

    private JButton btnDiscover;
    private JButton btnCalibrate;

    public CassetteFeederConfiguratorConfigurationWizard(CassetteFeederConfigurator feeder) {
        this.feeder = feeder;
        createUi();
    }

    private void createUi() {
        setLayout(new BoxLayout(this, BoxLayout.Y_AXIS));

        ColumnSpec labelCol = FormSpecs.DEFAULT_COLSPEC;
        ColumnSpec fieldCol = ColumnSpec.decode("max(26dlu;default)");
        ColumnSpec gap = FormSpecs.RELATED_GAP_COLSPEC;

        JPanel pnlBase = new JPanel(new FormLayout(
                new ColumnSpec[] { gap, labelCol, gap, fieldCol, gap, labelCol, gap, fieldCol, gap },
                new RowSpec[] {
                        FormSpecs.RELATED_GAP_ROWSPEC, FormSpecs.DEFAULT_ROWSPEC,
                        FormSpecs.RELATED_GAP_ROWSPEC, FormSpecs.DEFAULT_ROWSPEC,
                        FormSpecs.RELATED_GAP_ROWSPEC, FormSpecs.DEFAULT_ROWSPEC,
                        FormSpecs.RELATED_GAP_ROWSPEC, FormSpecs.DEFAULT_ROWSPEC,
                        FormSpecs.RELATED_GAP_ROWSPEC, FormSpecs.DEFAULT_ROWSPEC,
                        FormSpecs.RELATED_GAP_ROWSPEC, FormSpecs.DEFAULT_ROWSPEC,
                        FormSpecs.RELATED_GAP_ROWSPEC, FormSpecs.DEFAULT_ROWSPEC, }));
        pnlBase.setBorder(new TitledBorder("Baseplate Layout (shared by all CassetteAutoFeeders)"));

        pnlBase.add(new JLabel("Offset X"), "2,2,right,default");
        textFieldBaseplateOffsetX = new JTextField();
        pnlBase.add(textFieldBaseplateOffsetX, "4,2");
        pnlBase.add(new JLabel("Offset Y"), "6,2,right,default");
        textFieldBaseplateOffsetY = new JTextField();
        pnlBase.add(textFieldBaseplateOffsetY, "8,2");

        pnlBase.add(new JLabel("Offset Z"), "2,4,right,default");
        textFieldBaseplateOffsetZ = new JTextField();
        pnlBase.add(textFieldBaseplateOffsetZ, "4,4");
        pnlBase.add(new JLabel("Row Spacing"), "6,4,right,default");
        textFieldRowSpacing = new JTextField();
        pnlBase.add(textFieldRowSpacing, "8,4");

        pnlBase.add(new JLabel("Row Dir X"), "2,6,right,default");
        textFieldRowDirectionX = new JTextField();
        pnlBase.add(textFieldRowDirectionX, "4,6");
        pnlBase.add(new JLabel("Row Dir Y"), "6,6,right,default");
        textFieldRowDirectionY = new JTextField();
        pnlBase.add(textFieldRowDirectionY, "8,6");

        pnlBase.add(new JLabel("Total Rows"), "2,8,right,default");
        labelTotalRow = new JLabel();
        pnlBase.add(labelTotalRow, "4,8");
        pnlBase.add(new JLabel("Total Cols"), "6,8,right,default");
        textFieldTotalCol = new JTextField();
        pnlBase.add(textFieldTotalCol, "8,8");

        pnlBase.add(new JLabel("Continuous"), "2,10,right,default");
        textFieldContinuousRowCol = new JTextField();
        pnlBase.add(textFieldContinuousRowCol, "4,10");
        pnlBase.add(new JLabel("Skip"), "6,10,right,default");
        textFieldSkippedRowCol = new JTextField();
        pnlBase.add(textFieldSkippedRowCol, "8,10");

        pnlBase.add(new JLabel("Orientation"), "2,12,right,default");
        textFieldOrientation = new JTextField();
        pnlBase.add(textFieldOrientation, "4,12");

        checkBoxIsVerticalLayout = new JCheckBox("Vertical Layout");
        pnlBase.add(checkBoxIsVerticalLayout, "2,14,7,1");

        add(pnlBase);

        // ================================================================
        // Calibration: 2nd slot + 2nd-last slot + total slots -> offsets
        // ================================================================
        JPanel pnlCal = new JPanel(new FormLayout(
                new ColumnSpec[] { gap, labelCol, gap, fieldCol, gap, fieldCol, gap, fieldCol, gap,
                        FormSpecs.DEFAULT_COLSPEC },
                new RowSpec[] {
                        FormSpecs.RELATED_GAP_ROWSPEC, FormSpecs.DEFAULT_ROWSPEC,
                        FormSpecs.RELATED_GAP_ROWSPEC, FormSpecs.DEFAULT_ROWSPEC,
                        FormSpecs.RELATED_GAP_ROWSPEC, FormSpecs.DEFAULT_ROWSPEC,
                        FormSpecs.RELATED_GAP_ROWSPEC, FormSpecs.DEFAULT_ROWSPEC,
                        FormSpecs.RELATED_GAP_ROWSPEC, FormSpecs.DEFAULT_ROWSPEC,
                        FormSpecs.RELATED_GAP_ROWSPEC, FormSpecs.DEFAULT_ROWSPEC, }));
        pnlCal.setBorder(new TitledBorder("Calibration"));

        JLabel lblCalHelp = new JLabel("<html>Jog the camera to the <b>2nd</b> slot and Capture, "
                + "then to the <b>2nd-last</b> slot and Capture,<br/>"
                + "enter the total number of slots and press Calibrate.</html>");
        pnlCal.add(lblCalHelp, "2,2,9,1");

        pnlCal.add(new JLabel("X"), "4,4");
        pnlCal.add(new JLabel("Y"), "6,4");
        pnlCal.add(new JLabel("Z"), "8,4");

        JLabel lblCalSecond = new JLabel("2nd Slot");
        lblCalSecond.setToolTipText("<html>Camera location at the center of the <b>2nd</b> slot (index 1).<br/>"
                + "The first slot is usually out of camera reach, so we start here.</html>");
        pnlCal.add(lblCalSecond, "2,6,right,default");
        textFieldCalSecondX = new JTextField();
        pnlCal.add(textFieldCalSecondX, "4,6");
        textFieldCalSecondY = new JTextField();
        pnlCal.add(textFieldCalSecondY, "6,6");
        textFieldCalSecondZ = new JTextField();
        pnlCal.add(textFieldCalSecondZ, "8,6");
        pnlCal.add(new LocationButtonsPanel(
                textFieldCalSecondX, textFieldCalSecondY, textFieldCalSecondZ, null), "10,6");

        JLabel lblCalSecondLast = new JLabel("2nd-Last Slot");
        lblCalSecondLast.setToolTipText("<html>Camera location at the center of the <b>second-to-last</b> slot.<br/>"
                + "Together with the 2nd slot and the total slot count this defines<br/>"
                + "spacing, rail direction and the baseplate origin.</html>");
        pnlCal.add(lblCalSecondLast, "2,8,right,default");
        textFieldCalSecondLastX = new JTextField();
        pnlCal.add(textFieldCalSecondLastX, "4,8");
        textFieldCalSecondLastY = new JTextField();
        pnlCal.add(textFieldCalSecondLastY, "6,8");
        textFieldCalSecondLastZ = new JTextField();
        pnlCal.add(textFieldCalSecondLastZ, "8,8");
        pnlCal.add(new LocationButtonsPanel(
                textFieldCalSecondLastX, textFieldCalSecondLastY, textFieldCalSecondLastZ, null), "10,8");

        pnlCal.add(new JLabel("Total Slots"), "2,10,right,default");
        textFieldCalTotalSlots = new JTextField();
        pnlCal.add(textFieldCalTotalSlots, "4,10");

        btnCalibrate = new JButton(calibrateAction);
        btnCalibrate.setText("Calibrate Baseplate");
        btnCalibrate.setToolTipText("<html>Calculate Offset X/Y/Z, Row Spacing, Row Direction, Vertical Layout<br/>"
                + "and Total Rows from the 2nd slot, the 2nd-last slot and the total slot count.</html>");
        pnlCal.add(btnCalibrate, "2,12,7,1");

        add(pnlCal);

        JPanel pnlAct = new JPanel();
        pnlAct.setBorder(new TitledBorder("Actions"));
        pnlAct.setLayout(new BoxLayout(pnlAct, BoxLayout.Y_AXIS));
        btnDiscover = new JButton(discoverAction);
        btnDiscover.setText("Discover Feeders");
        btnDiscover.setToolTipText("<html>Query the firmware for all slots on this baseplate and<br/>"
                + "(re)create CassetteAutoFeeders using the layout above.</html>");
        pnlAct.add(btnDiscover);
        add(pnlAct);
    }

    @Override
    public void createBindings() {
        LengthConverter lc = new LengthConverter();
        IntegerConverter ic = new IntegerConverter();
        DoubleConverter dc = new DoubleConverter(Configuration.get().getLengthDisplayFormat());

        addWrappedBinding(feeder, "baseplateOffsetX", textFieldBaseplateOffsetX, "text", lc);
        addWrappedBinding(feeder, "baseplateOffsetY", textFieldBaseplateOffsetY, "text", lc);
        addWrappedBinding(feeder, "baseplateOffsetZ", textFieldBaseplateOffsetZ, "text", lc);
        addWrappedBinding(feeder, "rowSpacing", textFieldRowSpacing, "text", lc);
        addWrappedBinding(feeder, "rowDirectionX", textFieldRowDirectionX, "text", dc);
        addWrappedBinding(feeder, "rowDirectionY", textFieldRowDirectionY, "text", dc);
        addWrappedBinding(feeder, "totalCol", textFieldTotalCol, "text", ic);
        addWrappedBinding(feeder, "continuousRowCol", textFieldContinuousRowCol, "text", ic);
        addWrappedBinding(feeder, "skippedRowCol", textFieldSkippedRowCol, "text", ic);
        addWrappedBinding(feeder, "orientation", textFieldOrientation, "text", ic);
        addWrappedBinding(feeder, "isVerticalLayout", checkBoxIsVerticalLayout, "selected");

        // Total Rows is derived by the calibration; show it read-only here.
        bind(UpdateStrategy.READ, feeder, "totalRow", labelTotalRow, "text", ic);

        // Calibration inputs are persisted on the configurator so the panel is
        // repopulated when the wizard is reopened. Total Slots is the single
        // editable binding to totalRow.
        addWrappedBinding(feeder, "totalRow", textFieldCalTotalSlots, "text", ic);

        MutableLocationProxy secondSlotProxy = new MutableLocationProxy();
        bind(UpdateStrategy.READ_WRITE, feeder, "calibrationSecondSlot", secondSlotProxy, "location");
        addWrappedBinding(secondSlotProxy, "lengthX", textFieldCalSecondX, "text", lc);
        addWrappedBinding(secondSlotProxy, "lengthY", textFieldCalSecondY, "text", lc);
        addWrappedBinding(secondSlotProxy, "lengthZ", textFieldCalSecondZ, "text", lc);

        MutableLocationProxy secondLastSlotProxy = new MutableLocationProxy();
        bind(UpdateStrategy.READ_WRITE, feeder, "calibrationSecondLastSlot", secondLastSlotProxy,
                "location");
        addWrappedBinding(secondLastSlotProxy, "lengthX", textFieldCalSecondLastX, "text", lc);
        addWrappedBinding(secondLastSlotProxy, "lengthY", textFieldCalSecondLastY, "text", lc);
        addWrappedBinding(secondLastSlotProxy, "lengthZ", textFieldCalSecondLastZ, "text", lc);

        ComponentDecorators.decorateWithAutoSelectAndLengthConversion(textFieldBaseplateOffsetX);
        ComponentDecorators.decorateWithAutoSelectAndLengthConversion(textFieldBaseplateOffsetY);
        ComponentDecorators.decorateWithAutoSelectAndLengthConversion(textFieldBaseplateOffsetZ);
        ComponentDecorators.decorateWithAutoSelectAndLengthConversion(textFieldRowSpacing);
        ComponentDecorators.decorateWithAutoSelect(textFieldRowDirectionX);
        ComponentDecorators.decorateWithAutoSelect(textFieldRowDirectionY);
        ComponentDecorators.decorateWithAutoSelect(textFieldTotalCol);
        ComponentDecorators.decorateWithAutoSelect(textFieldContinuousRowCol);
        ComponentDecorators.decorateWithAutoSelect(textFieldSkippedRowCol);
        ComponentDecorators.decorateWithAutoSelect(textFieldOrientation);
        ComponentDecorators.decorateWithAutoSelectAndLengthConversion(textFieldCalSecondX);
        ComponentDecorators.decorateWithAutoSelectAndLengthConversion(textFieldCalSecondY);
        ComponentDecorators.decorateWithAutoSelectAndLengthConversion(textFieldCalSecondZ);
        ComponentDecorators.decorateWithAutoSelectAndLengthConversion(textFieldCalSecondLastX);
        ComponentDecorators.decorateWithAutoSelectAndLengthConversion(textFieldCalSecondLastY);
        ComponentDecorators.decorateWithAutoSelectAndLengthConversion(textFieldCalSecondLastZ);
        ComponentDecorators.decorateWithAutoSelect(textFieldCalTotalSlots);
    }

    // ================================================================
    // Actions
    // ================================================================

    private Action discoverAction = new AbstractAction("Discover Feeders") {
        @Override
        public void actionPerformed(ActionEvent e) {
            // Flush UI values (totalRow/totalCol etc.) into the configurator first.
            applyAction.actionPerformed(e);
            UiUtils.submitUiMachineTask(() -> feeder.discoverFeeders());
        }
    };

    private Action calibrateAction = new AbstractAction("Calibrate Baseplate") {
        @Override
        public void actionPerformed(ActionEvent e) {
            UiUtils.messageBoxOnException(() -> {
                // Flush the captured locations from the UI into the configurator.
                applyAction.actionPerformed(e);
                Location second = feeder.getCalibrationSecondSlot();
                Location secondLast = feeder.getCalibrationSecondLastSlot();
                int totalSlots;
                try {
                    totalSlots = Integer.parseInt(textFieldCalTotalSlots.getText().trim());
                }
                catch (NumberFormatException nfe) {
                    throw new IllegalArgumentException(
                            "Enter the total number of slots as an integer (got \""
                                    + textFieldCalTotalSlots.getText() + "\").");
                }
                feeder.calibrateBaseplateFromSlots(second, secondLast, totalSlots);
                // The baseplate fields are wrapped bindings, so the setters in
                // calibrateBaseplateFromSlots refresh the UI automatically.
            });
        }
    };
}
