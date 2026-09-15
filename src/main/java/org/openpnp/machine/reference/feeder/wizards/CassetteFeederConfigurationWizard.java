package org.openpnp.machine.reference.feeder.wizards;

import javax.swing.AbstractAction;
import javax.swing.Action;
import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JComboBox;
import javax.swing.JDialog;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JTextField;
import javax.swing.SwingUtilities;
import javax.swing.border.TitledBorder;

import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import org.jdesktop.beansbinding.AutoBinding.UpdateStrategy;
import org.openpnp.gui.MainFrame;
import org.openpnp.gui.components.ComponentDecorators;
import org.openpnp.gui.components.LocationButtonsPanel;
import org.openpnp.gui.support.AbstractConfigurationWizard;
import org.openpnp.gui.support.DoubleConverter;
import org.openpnp.gui.support.Icons;
import org.openpnp.gui.support.IntegerConverter;
import org.openpnp.gui.support.LengthConverter;
import org.openpnp.gui.support.LongConverter;
import org.openpnp.gui.support.MutableLocationProxy;
import org.openpnp.machine.reference.feeder.CassetteFeeder;
import org.openpnp.machine.reference.feeder.ReferencePushPullFeeder;
import org.openpnp.model.Configuration;
import org.openpnp.model.Length;
import org.openpnp.model.LengthUnit;
import org.openpnp.model.Location;
import org.openpnp.spi.Camera;
import org.openpnp.util.FeederVisionHelper.PipelineType;
import org.openpnp.util.MovableUtils;
import org.openpnp.util.UiUtils;
import org.openpnp.vision.pipeline.CvPipeline;
import org.openpnp.vision.pipeline.ui.CvPipelineEditor;
import org.openpnp.vision.pipeline.ui.CvPipelineEditorDialog;
import org.pmw.tinylog.Logger;

import com.jgoodies.forms.layout.ColumnSpec;
import com.jgoodies.forms.layout.FormLayout;
import com.jgoodies.forms.layout.FormSpecs;
import com.jgoodies.forms.layout.RowSpec;


@SuppressWarnings("serial")
public class CassetteFeederConfigurationWizard extends AbstractConfigurationWizard {
    private final CassetteFeeder feeder;

    private JPanel panelDiscover;
    private JPanel panelLocations;
    private JPanel panelPipeline;
    private JPanel panelLayout;
    private JPanel panelParam;

    private JButton btnDiscover;
    private JButton btnSave;
    private JButton btnUpdateOffsets;
    private JButton btnAutoSetup;
    private JButton btnShowVisionFeatures;
    private JButton btnEditPipeline;
    private JButton btnResetPipeline;
    private JComboBox<PipelineType> comboBoxPipelineType;

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
    private JComboBox<String> comboBoxTapeWidth;
    private JLabel labelSprocketToCavity;

    private JLabel labelTotalRow;
    private JLabel labelTotalCol;
    private JTextField textFieldTotalRow;
    private JTextField textFieldTotalCol;
    private JLabel labelRowSpacing;
    private JTextField textFieldRowSpacing;
    private JLabel labelSubType;
    private JTextField textFieldSubType;

    // Locations / holes
    private JLabel lblX;
    private JLabel lblY;
    private JLabel lblZ;
    private JLabel lblPickLocation;
    private JTextField textFieldPickLocationX;
    private JTextField textFieldPickLocationY;
    private JTextField textFieldPickLocationZ;
    private LocationButtonsPanel locationButtonsPanelPick;
    private JLabel lblHole1Location;
    private JTextField textFieldHole1LocationX;
    private JTextField textFieldHole1LocationY;
    private JTextField textFieldHole1LocationZ;
    private LocationButtonsPanel locationButtonsPanelHole1;
    private JLabel lblHole2Location;
    private JTextField textFieldHole2LocationX;
    private JTextField textFieldHole2LocationY;
    private JTextField textFieldHole2LocationZ;
    private LocationButtonsPanel locationButtonsPanelHole2;

    // Stage 1: manual baseplate calibration (3 captured slot centers)
    private JTextField textFieldSlotFirstX;
    private JTextField textFieldSlotFirstY;
    private JTextField textFieldSlotFirstZ;
    private LocationButtonsPanel locationButtonsPanelSlotFirst;
    private JTextField textFieldSlotSecondX;
    private JTextField textFieldSlotSecondY;
    private JTextField textFieldSlotSecondZ;
    private LocationButtonsPanel locationButtonsPanelSlotSecond;
    private JTextField textFieldSlotLastX;
    private JTextField textFieldSlotLastY;
    private JTextField textFieldSlotLastZ;
    private LocationButtonsPanel locationButtonsPanelSlotLast;
    private JButton btnCalibrateBaseplate;
    private JButton btnCalibrateFeederOffset;

    public CassetteFeederConfigurationWizard(CassetteFeeder feeder){
        super();
        this.feeder = feeder;

        JPanel panelFields = new JPanel();
        panelFields.setLayout(new BoxLayout(panelFields, BoxLayout.Y_AXIS));

        // ------------------------------------------------------------------
        // Discover panel (existing functionality)
        // ------------------------------------------------------------------
        panelDiscover = new JPanel();
        panelDiscover.setBorder(new TitledBorder(null, "Discover", TitledBorder.LEADING,
                TitledBorder.TOP, null, null));
        panelFields.add(panelDiscover);

        btnDiscover = new JButton(discoverFeederAction);
        btnDiscover.setToolTipText("Automatically discover feeders that has been plug into the base plate.");
        btnDiscover.setText("Discover Feeders");
        panelDiscover.add(btnDiscover);

        btnSave = new JButton(saveFeederAction);
        btnSave.setToolTipText("Save the current settings to feeder.");
        btnSave.setText("Save to Feeder");
        panelDiscover.add(btnSave);

        // ------------------------------------------------------------------
        // Calibration panel (Stage 1: baseplate, Stage 2: feeder offset)
        // ------------------------------------------------------------------
        JPanel panelCalibration = new JPanel();
        panelCalibration.setBorder(new TitledBorder(null, "Calibration", TitledBorder.LEADING,
                TitledBorder.TOP, null, null));
        panelCalibration.setLayout(new FormLayout(new ColumnSpec[] {
                FormSpecs.RELATED_GAP_COLSPEC,
                FormSpecs.DEFAULT_COLSPEC,
                FormSpecs.RELATED_GAP_COLSPEC,
                ColumnSpec.decode("max(26dlu;default)"),
                FormSpecs.RELATED_GAP_COLSPEC,
                ColumnSpec.decode("max(26dlu;default)"),
                FormSpecs.RELATED_GAP_COLSPEC,
                ColumnSpec.decode("max(26dlu;default)"),
                FormSpecs.RELATED_GAP_COLSPEC,
                ColumnSpec.decode("left:min"),
        },
        new RowSpec[] {
                FormSpecs.RELATED_GAP_ROWSPEC,
                FormSpecs.DEFAULT_ROWSPEC,
                FormSpecs.RELATED_GAP_ROWSPEC,
                FormSpecs.DEFAULT_ROWSPEC,
                FormSpecs.RELATED_GAP_ROWSPEC,
                FormSpecs.DEFAULT_ROWSPEC,
                FormSpecs.RELATED_GAP_ROWSPEC,
                FormSpecs.DEFAULT_ROWSPEC,
                FormSpecs.RELATED_GAP_ROWSPEC,
                FormSpecs.DEFAULT_ROWSPEC,
                FormSpecs.RELATED_GAP_ROWSPEC,
                FormSpecs.DEFAULT_ROWSPEC,
                FormSpecs.RELATED_GAP_ROWSPEC,
                FormSpecs.DEFAULT_ROWSPEC,
                FormSpecs.RELATED_GAP_ROWSPEC,
                FormSpecs.DEFAULT_ROWSPEC,
        }));
        panelFields.add(panelCalibration);

        JLabel lblStage1 = new JLabel("<html><b>Stage 1: Baseplate</b> &ndash; jog to slot centers, capture each, then Calibrate</html>");
        panelCalibration.add(lblStage1, "2, 2, 9, 1");

        JLabel lblCalX = new JLabel("X");
        panelCalibration.add(lblCalX, "4, 4");
        JLabel lblCalY = new JLabel("Y");
        panelCalibration.add(lblCalY, "6, 4");
        JLabel lblCalZ = new JLabel("Z");
        panelCalibration.add(lblCalZ, "8, 4");

        JLabel lblFirstSlot = new JLabel("2nd Slot Center");
        lblFirstSlot.setToolTipText("<html>Camera location at the center of the <b>2nd</b> slot (index 1).<br/>"
                + "Used with 3rd slot to derive spacing and rail direction.</html>");
        panelCalibration.add(lblFirstSlot, "2, 6, right, default");
        textFieldSlotFirstX = new JTextField();
        panelCalibration.add(textFieldSlotFirstX, "4, 6");
        textFieldSlotFirstX.setColumns(10);
        textFieldSlotFirstY = new JTextField();
        panelCalibration.add(textFieldSlotFirstY, "6, 6");
        textFieldSlotFirstY.setColumns(10);
        textFieldSlotFirstZ = new JTextField();
        panelCalibration.add(textFieldSlotFirstZ, "8, 6");
        textFieldSlotFirstZ.setColumns(10);
        locationButtonsPanelSlotFirst =
                new LocationButtonsPanel(textFieldSlotFirstX, textFieldSlotFirstY, textFieldSlotFirstZ, null);
        panelCalibration.add(locationButtonsPanelSlotFirst, "10, 6");

        JLabel lblSecondSlot = new JLabel("3rd Slot Center");
        lblSecondSlot.setToolTipText("<html>Camera location at the center of the <b>3rd</b> slot (index 2).<br/>"
                + "Together with the 2nd slot, defines the slot spacing (Row Spacing) and rail direction.</html>");
        panelCalibration.add(lblSecondSlot, "2, 8, right, default");
        textFieldSlotSecondX = new JTextField();
        panelCalibration.add(textFieldSlotSecondX, "4, 8");
 textFieldSlotSecondX.setColumns(10);
        textFieldSlotSecondY = new JTextField();
        panelCalibration.add(textFieldSlotSecondY, "6, 8");
        textFieldSlotSecondY.setColumns(10);
        textFieldSlotSecondZ = new JTextField();
        panelCalibration.add(textFieldSlotSecondZ, "8, 8");
        textFieldSlotSecondZ.setColumns(10);
        locationButtonsPanelSlotSecond =
                new LocationButtonsPanel(textFieldSlotSecondX, textFieldSlotSecondY, textFieldSlotSecondZ, null);
        panelCalibration.add(locationButtonsPanelSlotSecond, "10, 8");

        JLabel lblLastSlot = new JLabel("2nd Last Slot Center");
        lblLastSlot.setToolTipText("<html>Camera location at the center of the <b>second-to-last</b> slot on the rail.<br/>"
                + "Together with the 2nd slot, defines the total number of slots (Total Row).</html>");
        panelCalibration.add(lblLastSlot, "2, 10, right, default");
        textFieldSlotLastX = new JTextField();
        panelCalibration.add(textFieldSlotLastX, "4, 10");
        textFieldSlotLastX.setColumns(10);
        textFieldSlotLastY = new JTextField();
        panelCalibration.add(textFieldSlotLastY, "6, 10");
        textFieldSlotLastY.setColumns(10);
        textFieldSlotLastZ = new JTextField();
        panelCalibration.add(textFieldSlotLastZ, "8, 10");
        textFieldSlotLastZ.setColumns(10);
        locationButtonsPanelSlotLast =
                new LocationButtonsPanel(textFieldSlotLastX, textFieldSlotLastY, textFieldSlotLastZ, null);
        panelCalibration.add(locationButtonsPanelSlotLast, "10, 10");

        btnCalibrateBaseplate = new JButton(calibrateBaseplateAction);
        panelCalibration.add(btnCalibrateBaseplate, "2, 12, 5, 1");




        JLabel lblStage2 = new JLabel("<html><b>Stage 2: Feeder Offset</b> &ndash; set Hole 1 (Auto-Setup or capture), set Orientation, then Calibrate</html>");
        panelCalibration.add(lblStage2, "2, 14, 9, 1");

        btnCalibrateFeederOffset = new JButton(calibrateFeederOffsetAction);
        panelCalibration.add(btnCalibrateFeederOffset, "2, 16, 5, 1");

        // ------------------------------------------------------------------
        // Locations & Pipeline panel (NEW)
        // ------------------------------------------------------------------
        panelLocations = new JPanel();
        panelLocations.setBorder(new TitledBorder(null, "Pick Location, Holes & Pipeline", TitledBorder.LEADING,
                TitledBorder.TOP, null, null));
        panelLocations.setLayout(new FormLayout(new ColumnSpec[] {
                FormSpecs.RELATED_GAP_COLSPEC,
                FormSpecs.DEFAULT_COLSPEC,
                FormSpecs.RELATED_GAP_COLSPEC,
                ColumnSpec.decode("max(26dlu;default)"),
                FormSpecs.RELATED_GAP_COLSPEC,
                ColumnSpec.decode("max(26dlu;default)"),
                FormSpecs.RELATED_GAP_COLSPEC,
                ColumnSpec.decode("max(26dlu;default)"),
                FormSpecs.RELATED_GAP_COLSPEC,
                ColumnSpec.decode("left:min"),
                FormSpecs.RELATED_GAP_COLSPEC,
                FormSpecs.DEFAULT_COLSPEC,
        },
        new RowSpec[] {
                FormSpecs.RELATED_GAP_ROWSPEC,
                FormSpecs.DEFAULT_ROWSPEC,
                FormSpecs.RELATED_GAP_ROWSPEC,
                FormSpecs.DEFAULT_ROWSPEC,
                FormSpecs.RELATED_GAP_ROWSPEC,
                FormSpecs.DEFAULT_ROWSPEC,
                FormSpecs.RELATED_GAP_ROWSPEC,
                FormSpecs.DEFAULT_ROWSPEC,
                FormSpecs.RELATED_GAP_ROWSPEC,
                FormSpecs.DEFAULT_ROWSPEC,
                FormSpecs.RELATED_GAP_ROWSPEC,
                FormSpecs.DEFAULT_ROWSPEC,
        }));
        panelFields.add(panelLocations);

        btnShowVisionFeatures = new JButton(showVisionFeaturesAction);
        btnShowVisionFeatures.setText("Preview Vision Features");
        panelLocations.add(btnShowVisionFeatures, "2, 2, default, fill");

        btnAutoSetup = new JButton(autoSetupAction);
        panelLocations.add(btnAutoSetup, "4, 2, 5, 1");

        lblX = new JLabel("X");
        panelLocations.add(lblX, "4, 4");
        lblY = new JLabel("Y");
        panelLocations.add(lblY, "6, 4");
        lblZ = new JLabel("Z");
        panelLocations.add(lblZ, "8, 4");

        lblPickLocation = new JLabel("Pick Location");
        lblPickLocation.setToolTipText("<html>"
                + "Center of the part pocket for the current row/column.<br/>"
                + "Jog the camera to the pocket center and click the<br/>"
                + "<em>Capture Camera</em> button, then press <strong>Auto-Setup</strong>.</html>");
        panelLocations.add(lblPickLocation, "2, 6, right, default");

        textFieldPickLocationX = new JTextField();
        panelLocations.add(textFieldPickLocationX, "4, 6");
        textFieldPickLocationX.setColumns(10);
        textFieldPickLocationY = new JTextField();
        panelLocations.add(textFieldPickLocationY, "6, 6");
        textFieldPickLocationY.setColumns(10);
        textFieldPickLocationZ = new JTextField();
        panelLocations.add(textFieldPickLocationZ, "8, 6");
        textFieldPickLocationZ.setColumns(10);
        locationButtonsPanelPick =
                new LocationButtonsPanel(textFieldPickLocationX, textFieldPickLocationY, textFieldPickLocationZ, null);
        panelLocations.add(locationButtonsPanelPick, "10, 6");

        lblHole1Location = new JLabel("Hole 1 Location");
        lblHole1Location.setToolTipText("<html>Sprocket hole 1 (auto-detected by Auto-Setup, or set manually).<br/>"
                + "Holes 1 and 2 should bracket the pick pocket along the tape axis.</html>");
        panelLocations.add(lblHole1Location, "2, 8, right, default");
        textFieldHole1LocationX = new JTextField();
        panelLocations.add(textFieldHole1LocationX, "4, 8");
        textFieldHole1LocationX.setColumns(10);
        textFieldHole1LocationY = new JTextField();
        panelLocations.add(textFieldHole1LocationY, "6, 8");
        textFieldHole1LocationY.setColumns(10);
        textFieldHole1LocationZ = new JTextField();
        panelLocations.add(textFieldHole1LocationZ, "8, 8");
        textFieldHole1LocationZ.setColumns(10);
        locationButtonsPanelHole1 =
                new LocationButtonsPanel(textFieldHole1LocationX, textFieldHole1LocationY, textFieldHole1LocationZ, null);
        panelLocations.add(locationButtonsPanelHole1, "10, 8");

        lblHole2Location = new JLabel("Hole 2 Location");
        lblHole2Location.setToolTipText("<html>Sprocket hole 2 (auto-detected by Auto-Setup, or set manually).</html>");
        panelLocations.add(lblHole2Location, "2, 10, right, default");
        textFieldHole2LocationX = new JTextField();
        panelLocations.add(textFieldHole2LocationX, "4, 10");
        textFieldHole2LocationX.setColumns(10);
        textFieldHole2LocationY = new JTextField();
        panelLocations.add(textFieldHole2LocationY, "6, 10");
        textFieldHole2LocationY.setColumns(10);
        textFieldHole2LocationZ = new JTextField();
        panelLocations.add(textFieldHole2LocationZ, "8, 10");
        textFieldHole2LocationZ.setColumns(10);
        locationButtonsPanelHole2 =
                new LocationButtonsPanel(textFieldHole2LocationX, textFieldHole2LocationY, textFieldHole2LocationZ, null);
        panelLocations.add(locationButtonsPanelHole2, "10, 10");

        // Pipeline selector + edit / reset
        JLabel lblPipeline = new JLabel("Pipeline");
        lblPipeline.setToolTipText("<html>"
                + "Choose the vision pipeline type used to detect sprocket holes.<br/>"
                + "<strong>ColorKeyed</strong>: the background under the holes must be of a vivid color.<br/>"
                + "<strong>CircularSymmetry</strong>: holes are detected by their circular shape.<br/>"
                + "Use <em>Edit Pipeline</em> to tune detection for your camera and lighting.</html>");
        panelLocations.add(lblPipeline, "2, 12, right, default");
        comboBoxPipelineType = new JComboBox<>(PipelineType.values());
        panelLocations.add(comboBoxPipelineType, "4, 12, 3, 1");
        btnEditPipeline = new JButton(editPipelineAction);
        panelLocations.add(btnEditPipeline, "8, 12");
        btnResetPipeline = new JButton(resetPipelineAction);
        panelLocations.add(btnResetPipeline, "10, 12");

        // ------------------------------------------------------------------
        // Baseplate layout panel (existing)
        // ------------------------------------------------------------------
        panelLayout = new JPanel();
        panelLayout.setBorder(new TitledBorder(null, "Baseplate Layout", TitledBorder.LEADING,
                TitledBorder.TOP, null, null));
        panelLayout.setLayout(new GridLayout(0, 6));

        labelIsLayoutVertical = new JLabel("Vertical layout (Col mode)");
        panelLayout.add(labelIsLayoutVertical);
        checkBoxIsVertical = new JCheckBox();
        panelLayout.add(checkBoxIsVertical);

        labelContinuousRowCol = new JLabel("Continuous Row/Col");
        panelLayout.add(labelContinuousRowCol);
        textFieldContinuousRowCol = new JTextField();
        panelLayout.add(textFieldContinuousRowCol);
        labelSkippedRowCol = new JLabel("Skipped Row/Col");
        panelLayout.add(labelSkippedRowCol);
        textFieldSkippedRowCol = new JTextField();
        panelLayout.add(textFieldSkippedRowCol);

        btnUpdateOffsets = new JButton(updateOffsetsAction);
        btnUpdateOffsets.setToolTipText("Update Offset X/Y/Z values based on currently set Hole 1 and Pick positions.");
        btnUpdateOffsets.setText("Update Offsets from Positions");
        panelLayout.add(btnUpdateOffsets, "1, 1, 6, 1");

        labelOffsetX = new JLabel("Offset X");
        panelLayout.add(labelOffsetX);
        textFieldBaseplateOffsetX = new JTextField();
        panelLayout.add(textFieldBaseplateOffsetX);

        labelOffsetY = new JLabel("Offset Y");
        panelLayout.add(labelOffsetY);
        textFieldBaseplateOffsetY = new JTextField();
        panelLayout.add(textFieldBaseplateOffsetY);

        labelOffsetZ = new JLabel("Offset Z");
        panelLayout.add(labelOffsetZ);
        textFieldBaseplateOffsetZ = new JTextField();
        panelLayout.add(textFieldBaseplateOffsetZ);

        labelTotalRow = new JLabel("Total Row");
        panelLayout.add(labelTotalRow);
        textFieldTotalRow = new JTextField();
        panelLayout.add(textFieldTotalRow);
        labelTotalCol = new JLabel("Total Column");
        panelLayout.add(labelTotalCol);
        textFieldTotalCol = new JTextField();
        panelLayout.add(textFieldTotalCol);

        labelRowSpacing = new JLabel("Row Spacing");
        panelLayout.add(labelRowSpacing);
        textFieldRowSpacing = new JTextField();
        panelLayout.add(textFieldRowSpacing);

        labelSubType = new JLabel("Sub Type");
        panelLayout.add(labelSubType);
        textFieldSubType = new JTextField();
        panelLayout.add(textFieldSubType);

        panelFields.add(panelLayout);

        // ------------------------------------------------------------------
        // Feeder parameters panel (existing)
        // ------------------------------------------------------------------
        panelParam = new JPanel();
        panelParam.setBorder(new TitledBorder(null, "Feeder Parameters", TitledBorder.LEADING,
                TitledBorder.TOP, null, null));
        panelParam.setLayout(new GridLayout(0, 8));
        labelOrientation = new JLabel("Orientation (deg)");
        panelParam.add(labelOrientation);
        textFieldOrientation = new JTextField();
        textFieldOrientation.setToolTipText("<html>"
                + "Sets the physical orientation of the baseplate in degrees.<br/>"
                + "Must be one of 0, 90, 180, 270.<br/><br/>"
                + "<strong>Hole-to-pick direction</strong>:<br/>"
                + "&nbsp;&nbsp;Orientation 0 → holes are <em>ahead</em> (in +X after rotation)<br/>"
                + "&nbsp;&nbsp;Orientation 180 → holes are <em>behind</em> (in −X after rotation)<br/>"
                + "Check the Auto-Setup log for the detected lateral-offset side<br/>"
                + "(&#39;left of axis&#39; / &#39;right of axis&#39;). If the pick lands on the<br/>"
                + "wrong side, flip orientation (e.g., 0 ↔ 180) and re-run Auto-Setup.</html>");
        panelParam.add(textFieldOrientation);

        JLabel labelTapeWidth = new JLabel("Tape Width (mm)");
        panelParam.add(labelTapeWidth);
        comboBoxTapeWidth = new JComboBox<>(new String[] {
                "8", "12", "16", "24", "32", "44", "56"
        });
        comboBoxTapeWidth.setToolTipText("<html>"
                + "EIA-481 tape width. Changing this automatically sets<br/>"
                + "the sprocket-to-cavity distance (F dimension).</html>");
        panelParam.add(comboBoxTapeWidth);

        labelSprocketToCavity = new JLabel("F=3.5mm");
        labelSprocketToCavity.setToolTipText("EIA-481 F dimension: auto-selected from tape width.");
        panelParam.add(labelSprocketToCavity);

        panelFields.add(panelParam);

        contentPanel.add(panelFields);

        // Load persisted slot capture positions into the text fields
    }

    @Override
    protected void saveToModel() {
        Logger.debug("Saving model");
        super.saveToModel();
    }

    // ----------------------------------------------------------------------
    // Actions
    // ----------------------------------------------------------------------

    private Action autoSetupAction =
            new AbstractAction("Auto-Setup", Icons.captureCamera) {
        {
            putValue(Action.SHORT_DESCRIPTION, "<html>"
                    + "Detect sprocket holes around the Pick Location using the configured pipeline,<br/>"
                    + "set Hole 1 / Hole 2 and recalculate the baseplate offsets.<br/>"
                    + "<strong>Before pressing</strong>: jog the camera to the part pocket, click the<br/>"
                    + "Capture Camera button next to Pick Location.</html>");
        }

        @Override
        public void actionPerformed(ActionEvent e) {
            UiUtils.messageBoxOnException(() -> {
                applyAction.actionPerformed(e);
                UiUtils.submitUiMachineTask(() -> {
                    feeder.autoSetup();
                });
            });
        }
    };

    private Action showVisionFeaturesAction =
            new AbstractAction("Preview Vision Features") {
        {
            putValue(Action.SHORT_DESCRIPTION, "<html>"
                    + "Run the vision pipeline and overlay the detected holes on the camera view.<br/>"
                    + "Green = all detected holes, cyan = selected sprocket-hole pair,<br/>"
                    + "yellow crosshair = Pick Location. Use this while tuning the pipeline.</html>");
        }

        @Override
        public void actionPerformed(ActionEvent e) {
            UiUtils.submitUiMachineTask(() -> {
                feeder.showFeatures();
            });
        }
    };

    private Action editPipelineAction =
            new AbstractAction("Edit Pipeline") {
        {
            putValue(Action.SHORT_DESCRIPTION,
                    "Edit the vision pipeline used for sprocket-hole detection.");
        }

        @Override
        public void actionPerformed(ActionEvent e) {
            UiUtils.messageBoxOnException(() -> {
                editPipeline();
            });
        }
    };

    private Action resetPipelineAction =
            new AbstractAction("Reset Pipeline") {
        {
            putValue(Action.SHORT_DESCRIPTION,
                    "Reset the pipeline to the default for the selected type.");
        }

        @Override
        public void actionPerformed(ActionEvent e) {
            PipelineType type = (PipelineType) comboBoxPipelineType.getSelectedItem();
            int result = JOptionPane.showConfirmDialog(getTopLevelAncestor(),
                    "This will reset the pipeline to the " + type + " default. Are you sure?",
                    null, JOptionPane.YES_NO_OPTION, JOptionPane.WARNING_MESSAGE);
            if (result == JOptionPane.YES_OPTION) {
                applyAction.actionPerformed(e);
                UiUtils.messageBoxOnException(() -> {
                    feeder.resetPipeline(type);
                });
            }
        }
    };

    private Action updateOffsetsAction =
            new AbstractAction("UpdateOffsets") {
        {
            putValue(Action.SHORT_DESCRIPTION,
                    "<html>Recalculates Offset X, Offset Y, Offset Z from the currently set<br/>"
                    + "Hole 1 Location and Pick Location. Use this after manually adjusting<br/>"
                    + "positions so that other feeders on the same baseplate will be correct.</html>");
        }

        @Override
        public void actionPerformed(ActionEvent e) {
            UiUtils.messageBoxOnException(() -> {
                applyAction.actionPerformed(e);
                feeder.calculateBaseplateOffsetsFromPositions();
                SwingUtilities.invokeLater(() -> {
                    textFieldBaseplateOffsetX.setText(
                        new LengthConverter().convertForward(feeder.getBaseplateOffsetX()));
                    textFieldBaseplateOffsetY.setText(
                        new LengthConverter().convertForward(feeder.getBaseplateOffsetY()));
                    textFieldBaseplateOffsetZ.setText(
                        new LengthConverter().convertForward(feeder.getBaseplateOffsetZ()));
                });
            });
        }
    };

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

    private Location parseLocationFromFields(JTextField x, JTextField y, JTextField z) {
        LengthUnit su = Configuration.get().getSystemUnits();
        Length lx = Length.parseWithDefaultUnits(x.getText(), su);
        Length ly = Length.parseWithDefaultUnits(y.getText(), su);
        Length lz = Length.parseWithDefaultUnits(z.getText(), su);
        if (lx == null) {
            lx = new Length(0, su);
        }
        else {
            lx = lx.convertToUnits(su);
        }
        if (ly == null) {
            ly = new Length(0, su);
        }
        else {
            ly = ly.convertToUnits(su);
        }
        if (lz == null) {
            lz = new Length(0, su);
        }
        else {
            lz = lz.convertToUnits(su);
        }
        return new Location(su, lx.getValue(), ly.getValue(), lz.getValue(), 0);
    }

    private Action calibrateBaseplateAction =
            new AbstractAction("Calibrate Baseplate", Icons.captureCamera) {
        {
            putValue(Action.SHORT_DESCRIPTION, "<html>"
                    + "Calculate baseplate Offset X/Y/Z, Row Spacing, Vertical Layout and Total Row<br/>"
                    + "from the three captured slot centers (2nd / 3rd / 2nd-last).</html>");
        }

        @Override
        public void actionPerformed(ActionEvent e) {
            UiUtils.messageBoxOnException(() -> {
                Location second = parseLocationFromFields(textFieldSlotFirstX, textFieldSlotFirstY, textFieldSlotFirstZ);
                Location third = parseLocationFromFields(textFieldSlotSecondX, textFieldSlotSecondY, textFieldSlotSecondZ);
                Location secondLast = parseLocationFromFields(textFieldSlotLastX, textFieldSlotLastY, textFieldSlotLastZ);
                // Persist captured positions
                feeder.capturedSlotSecond = second;
                feeder.capturedSlotThird = third;
                feeder.capturedSlotSecondLast = secondLast;
                feeder.calibrateBaseplateFromInnerSlots(second, third, secondLast);
                SwingUtilities.invokeLater(() -> {
                });
            });
        }
    };

    private Action calibrateFeederOffsetAction =
            new AbstractAction("Calibrate Feeder Offset", Icons.captureCamera) {
        {
            putValue(Action.SHORT_DESCRIPTION, "<html>"
                    + "Calculate the per-feeder contact offset (ox, oy) from Hole 1 Location and<br/>"
                    + "the baseplate model, then derive Pick / Hole 2. Use after Stage 1 and after<br/>"
                    + "Hole 1 is set (Auto-Setup, Preview, or manual capture). The result is saved<br/>"
                    + "to the feeder with Save to Feeder.</html>");
        }

        @Override
        public void actionPerformed(ActionEvent e) {
            applyAction.actionPerformed(e);
            UiUtils.messageBoxOnException(() -> {
                feeder.calibrateFeederOffsetFromHole1();
                LengthConverter lc = new LengthConverter();
                SwingUtilities.invokeLater(() -> {
                    textFieldHole2LocationX.setText(lc.convertForward(feeder.getHole2Location().getLengthX()));
                    textFieldHole2LocationY.setText(lc.convertForward(feeder.getHole2Location().getLengthY()));
                    textFieldHole2LocationZ.setText(lc.convertForward(feeder.getHole2Location().getLengthZ()));
                });
            });
        }
    };

    private void editPipeline() throws Exception {
        Camera camera = feeder.getCamera();
        CvPipeline pipeline = feeder.getCvPipeline(camera, false, false, true);
        CvPipelineEditor editor = new CvPipelineEditor(pipeline);
        JDialog dialog = new CvPipelineEditorDialog(MainFrame.get(), feeder.getName() + " Pipeline", editor);
        dialog.setVisible(true);
    }

    // ----------------------------------------------------------------------
    // Bindings
    // ----------------------------------------------------------------------

    @Override
    public void createBindings() {
        LengthConverter lengthConverter = new LengthConverter();
        IntegerConverter intConverter = new IntegerConverter();
        DoubleConverter doubleConverter =
                new DoubleConverter(Configuration.get().getLengthDisplayFormat());

        // Pick Location
        MutableLocationProxy pickLocation = new MutableLocationProxy();
        bind(UpdateStrategy.READ_WRITE, feeder, "location", pickLocation, "location");
        addWrappedBinding(pickLocation, "lengthX", textFieldPickLocationX, "text", lengthConverter);
        addWrappedBinding(pickLocation, "lengthY", textFieldPickLocationY, "text", lengthConverter);
        addWrappedBinding(pickLocation, "lengthZ", textFieldPickLocationZ, "text", lengthConverter);

        // Hole 1 Location
        MutableLocationProxy hole1Location = new MutableLocationProxy();
        bind(UpdateStrategy.READ_WRITE, feeder, "hole1Location", hole1Location, "location");
        addWrappedBinding(hole1Location, "lengthX", textFieldHole1LocationX, "text", lengthConverter);
        addWrappedBinding(hole1Location, "lengthY", textFieldHole1LocationY, "text", lengthConverter);
        addWrappedBinding(hole1Location, "lengthZ", textFieldHole1LocationZ, "text", lengthConverter);

        // Hole 2 Location
        MutableLocationProxy hole2Location = new MutableLocationProxy();
        bind(UpdateStrategy.READ_WRITE, feeder, "hole2Location", hole2Location, "location");
        addWrappedBinding(hole2Location, "lengthX", textFieldHole2LocationX, "text", lengthConverter);
        addWrappedBinding(hole2Location, "lengthY", textFieldHole2LocationY, "text", lengthConverter);
        addWrappedBinding(hole2Location, "lengthZ", textFieldHole2LocationZ, "text", lengthConverter);

        // Pipeline
        addWrappedBinding(feeder, "pipelineType", comboBoxPipelineType, "selectedItem");

        // Baseplate layout
        addWrappedBinding(feeder, "baseplateOffsetX", textFieldBaseplateOffsetX, "text", lengthConverter);
        addWrappedBinding(feeder, "baseplateOffsetY", textFieldBaseplateOffsetY, "text", lengthConverter);
        addWrappedBinding(feeder, "baseplateOffsetZ", textFieldBaseplateOffsetZ, "text", lengthConverter);
        addWrappedBinding(feeder, "continuousRowCol", textFieldContinuousRowCol, "text", intConverter);
        addWrappedBinding(feeder, "skippedRowCol", textFieldSkippedRowCol, "text", intConverter);
        addWrappedBinding(feeder, "isVerticalLayout", checkBoxIsVertical, "selected");
        addWrappedBinding(feeder, "orientation", textFieldOrientation, "text", intConverter);
        addWrappedBinding(feeder, "tapeWidthMm", comboBoxTapeWidth, "selectedItem");
        // Update the F label whenever sprocketHoleToCavityMm changes.
        feeder.addPropertyChangeListener("sprocketHoleToCavityMm", e -> {
            labelSprocketToCavity.setText(String.format("F=%.2fmm", feeder.getSprocketHoleToCavityMm()));
        });
        addWrappedBinding(feeder, "totalRow", textFieldTotalRow, "text", intConverter);
        addWrappedBinding(feeder, "totalCol", textFieldTotalCol, "text", intConverter);
        addWrappedBinding(feeder, "rowSpacing", textFieldRowSpacing, "text", lengthConverter);
        addWrappedBinding(feeder, "subType", textFieldSubType, "text", intConverter);

        ComponentDecorators.decorateWithAutoSelectAndLengthConversion(textFieldPickLocationX);
        ComponentDecorators.decorateWithAutoSelectAndLengthConversion(textFieldPickLocationY);
        ComponentDecorators.decorateWithAutoSelectAndLengthConversion(textFieldPickLocationZ);
        ComponentDecorators.decorateWithAutoSelectAndLengthConversion(textFieldHole1LocationX);
        ComponentDecorators.decorateWithAutoSelectAndLengthConversion(textFieldHole1LocationY);
        ComponentDecorators.decorateWithAutoSelectAndLengthConversion(textFieldHole1LocationZ);
        ComponentDecorators.decorateWithAutoSelectAndLengthConversion(textFieldHole2LocationX);
        ComponentDecorators.decorateWithAutoSelectAndLengthConversion(textFieldHole2LocationY);
        ComponentDecorators.decorateWithAutoSelectAndLengthConversion(textFieldHole2LocationZ);
        ComponentDecorators.decorateWithAutoSelectAndLengthConversion(textFieldBaseplateOffsetX);
        ComponentDecorators.decorateWithAutoSelectAndLengthConversion(textFieldBaseplateOffsetY);
        ComponentDecorators.decorateWithAutoSelectAndLengthConversion(textFieldBaseplateOffsetZ);
        ComponentDecorators.decorateWithAutoSelectAndLengthConversion(textFieldRowSpacing);

        ComponentDecorators.decorateWithAutoSelectAndLengthConversion(textFieldSlotFirstX);
        ComponentDecorators.decorateWithAutoSelectAndLengthConversion(textFieldSlotFirstY);
        ComponentDecorators.decorateWithAutoSelectAndLengthConversion(textFieldSlotFirstZ);
        ComponentDecorators.decorateWithAutoSelectAndLengthConversion(textFieldSlotSecondX);
        ComponentDecorators.decorateWithAutoSelectAndLengthConversion(textFieldSlotSecondY);
        ComponentDecorators.decorateWithAutoSelectAndLengthConversion(textFieldSlotSecondZ);
        ComponentDecorators.decorateWithAutoSelectAndLengthConversion(textFieldSlotLastX);
        ComponentDecorators.decorateWithAutoSelectAndLengthConversion(textFieldSlotLastY);
        ComponentDecorators.decorateWithAutoSelectAndLengthConversion(textFieldSlotLastZ);
    }
}
