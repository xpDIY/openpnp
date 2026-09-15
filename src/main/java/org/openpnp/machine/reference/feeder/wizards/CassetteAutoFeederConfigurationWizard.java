package org.openpnp.machine.reference.feeder.wizards;

import java.awt.event.ActionEvent;

import javax.swing.AbstractAction;
import javax.swing.Action;
import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextField;
import javax.swing.SwingUtilities;
import javax.swing.border.TitledBorder;

import org.jdesktop.beansbinding.AutoBinding.UpdateStrategy;
import org.openpnp.gui.components.ComponentDecorators;
import org.openpnp.gui.components.LocationButtonsPanel;
import org.openpnp.gui.support.AbstractConfigurationWizard;
import org.openpnp.gui.support.DoubleConverter;
import org.openpnp.gui.support.IntegerConverter;
import org.openpnp.gui.support.LengthConverter;
import org.openpnp.gui.support.MutableLocationProxy;
import org.openpnp.machine.reference.feeder.CassetteAutoFeeder;
import org.openpnp.machine.reference.feeder.CassetteFeederConfigurator;
import org.openpnp.model.Configuration;
import org.openpnp.model.LengthUnit;
import org.openpnp.spi.Actuator;
import org.openpnp.util.UiUtils;
import org.pmw.tinylog.Logger;

import com.jgoodies.forms.layout.ColumnSpec;
import com.jgoodies.forms.layout.FormLayout;
import com.jgoodies.forms.layout.FormSpecs;
import com.jgoodies.forms.layout.RowSpec;

@SuppressWarnings("serial")
public class CassetteAutoFeederConfigurationWizard extends AbstractConfigurationWizard {
    private final CassetteAutoFeeder feeder;

    // Slot fields
    private JTextField textFieldRow, textFieldCol, textFieldSubType;
    // Per-feeder firmware contact offsets (OX/OY in mm) and height, computed by
    // calculateOffsetsFromRefHole and flashed by Save To Feeder.
    private JTextField textFieldContactOX, textFieldContactOY, textFieldFeederHeight;

    // Tape fields
    private JTextField textFieldPartPitch;
    private JTextField textFieldTapeWidth;
    private JLabel labelPartsPerHole;

    // Pick location
    private JTextField textFieldPickX, textFieldPickY, textFieldPickZ;
    private LocationButtonsPanel pickLocationButtonsPanel;

    // Reference hole location
    private JTextField textFieldRefHoleX, textFieldRefHoleY, textFieldRefHoleZ;
    private LocationButtonsPanel refHoleLocationButtonsPanel;

    // Last hole location
    private JTextField textFieldLastHoleX, textFieldLastHoleY, textFieldLastHoleZ;
    private LocationButtonsPanel lastHoleLocationButtonsPanel;

    // Action buttons
    private JButton btnCalculateOffsets;
    private JButton btnSaveToFeeder;

    public CassetteAutoFeederConfigurationWizard(CassetteAutoFeeder feeder) {
        this.feeder = feeder;
        createUi();
    }

    private void createUi() {
        setLayout(new BoxLayout(this, BoxLayout.Y_AXIS));

        ColumnSpec labelCol = FormSpecs.DEFAULT_COLSPEC;
        ColumnSpec fieldCol = ColumnSpec.decode("max(26dlu;default)");
        ColumnSpec gap = FormSpecs.RELATED_GAP_COLSPEC;

        // ================================================================
        // Tape Settings
        // ================================================================
        JPanel pnlTape = new JPanel(new FormLayout(
                new ColumnSpec[] { gap, labelCol, gap, fieldCol, gap, labelCol, gap, fieldCol, gap },
                new RowSpec[] {
                        FormSpecs.RELATED_GAP_ROWSPEC, FormSpecs.DEFAULT_ROWSPEC,
                        FormSpecs.RELATED_GAP_ROWSPEC, FormSpecs.DEFAULT_ROWSPEC, }));
        pnlTape.setBorder(new TitledBorder("Tape"));

        pnlTape.add(new JLabel("Part Pitch"), "2,2,right,default");
        textFieldPartPitch = new JTextField();
        pnlTape.add(textFieldPartPitch, "4,2");

        pnlTape.add(new JLabel("Parts Per Hole"), "6,2,right,default");
        labelPartsPerHole = new JLabel("1");
        pnlTape.add(labelPartsPerHole, "8,2");

        pnlTape.add(new JLabel("Tape Width"), "2,4,right,default");
        textFieldTapeWidth = new JTextField();
        pnlTape.add(textFieldTapeWidth, "4,4");

        add(pnlTape);

        // ================================================================
        // Slot
        // ================================================================
        JPanel pnlSlot = new JPanel(new FormLayout(
                new ColumnSpec[] { gap, labelCol, gap, fieldCol, gap, labelCol, gap, fieldCol, gap },
                new RowSpec[] {
                        FormSpecs.RELATED_GAP_ROWSPEC, FormSpecs.DEFAULT_ROWSPEC,
                        FormSpecs.RELATED_GAP_ROWSPEC, FormSpecs.DEFAULT_ROWSPEC,
                        FormSpecs.RELATED_GAP_ROWSPEC, FormSpecs.DEFAULT_ROWSPEC,
                        FormSpecs.RELATED_GAP_ROWSPEC, FormSpecs.DEFAULT_ROWSPEC, }));
        pnlSlot.setBorder(new TitledBorder("Slot"));

        pnlSlot.add(new JLabel("Row"), "2,2,right,default");
        textFieldRow = new JTextField();
        pnlSlot.add(textFieldRow, "4,2");
        pnlSlot.add(new JLabel("Col"), "6,2,right,default");
        textFieldCol = new JTextField();
        pnlSlot.add(textFieldCol, "8,2");

        pnlSlot.add(new JLabel("SubType"), "2,4,right,default");
        textFieldSubType = new JTextField();
        pnlSlot.add(textFieldSubType, "4,4");

        // OX / OY are the per-feeder contact offsets (mm) flashed to firmware.
        pnlSlot.add(new JLabel("OX (mm)"), "2,6,right,default");
        textFieldContactOX = new JTextField();
        pnlSlot.add(textFieldContactOX, "4,6");
        pnlSlot.add(new JLabel("OY (mm)"), "6,6,right,default");
        textFieldContactOY = new JTextField();
        pnlSlot.add(textFieldContactOY, "8,6");

        pnlSlot.add(new JLabel("Height"), "2,8,right,default");
        textFieldFeederHeight = new JTextField();
        pnlSlot.add(textFieldFeederHeight, "4,8");

        add(pnlSlot);

        // ================================================================
        // Locations (with jog/capture buttons)
        // ================================================================
        JPanel pnlLoc = new JPanel(new FormLayout(
                new ColumnSpec[] { gap, labelCol, gap, fieldCol, gap, fieldCol, gap, fieldCol, gap,
                        FormSpecs.DEFAULT_COLSPEC },
                new RowSpec[] {
                        FormSpecs.RELATED_GAP_ROWSPEC, FormSpecs.DEFAULT_ROWSPEC,
                        FormSpecs.RELATED_GAP_ROWSPEC, FormSpecs.DEFAULT_ROWSPEC,
                        FormSpecs.RELATED_GAP_ROWSPEC, FormSpecs.DEFAULT_ROWSPEC,
                        FormSpecs.RELATED_GAP_ROWSPEC, FormSpecs.DEFAULT_ROWSPEC,
                        FormSpecs.RELATED_GAP_ROWSPEC, FormSpecs.DEFAULT_ROWSPEC,
                        FormSpecs.RELATED_GAP_ROWSPEC, FormSpecs.DEFAULT_ROWSPEC, }));
        pnlLoc.setBorder(new TitledBorder("Locations"));

        // Column headers
        pnlLoc.add(new JLabel("X"), "4,2");
        pnlLoc.add(new JLabel("Y"), "6,2");
        pnlLoc.add(new JLabel("Z"), "8,2");

        // Pick location
        pnlLoc.add(new JLabel("Pick"), "2,4,right,default");
        textFieldPickX = new JTextField();
        pnlLoc.add(textFieldPickX, "4,4");
        textFieldPickY = new JTextField();
        pnlLoc.add(textFieldPickY, "6,4");
        textFieldPickZ = new JTextField();
        pnlLoc.add(textFieldPickZ, "8,4");
        pickLocationButtonsPanel = new LocationButtonsPanel(
                textFieldPickX, textFieldPickY, textFieldPickZ, null);
        pnlLoc.add(pickLocationButtonsPanel, "10,4");

        // Reference hole
        pnlLoc.add(new JLabel("Ref Hole"), "2,6,right,default");
        textFieldRefHoleX = new JTextField();
        pnlLoc.add(textFieldRefHoleX, "4,6");
        textFieldRefHoleY = new JTextField();
        pnlLoc.add(textFieldRefHoleY, "6,6");
        textFieldRefHoleZ = new JTextField();
        pnlLoc.add(textFieldRefHoleZ, "8,6");
        refHoleLocationButtonsPanel = new LocationButtonsPanel(
                textFieldRefHoleX, textFieldRefHoleY, textFieldRefHoleZ, null);
        pnlLoc.add(refHoleLocationButtonsPanel, "10,6");

        // Last hole
        pnlLoc.add(new JLabel("Last Hole"), "2,8,right,default");
        textFieldLastHoleX = new JTextField();
        pnlLoc.add(textFieldLastHoleX, "4,8");
        textFieldLastHoleY = new JTextField();
        pnlLoc.add(textFieldLastHoleY, "6,8");
        textFieldLastHoleZ = new JTextField();
        pnlLoc.add(textFieldLastHoleZ, "8,8");
        lastHoleLocationButtonsPanel = new LocationButtonsPanel(
                textFieldLastHoleX, textFieldLastHoleY, textFieldLastHoleZ, null);
        pnlLoc.add(lastHoleLocationButtonsPanel, "10,8");

        add(pnlLoc);

        // ================================================================
        // Actions
        // ================================================================
        JPanel pnlAct = new JPanel();
        pnlAct.setBorder(new TitledBorder("Actions"));
        pnlAct.setLayout(new BoxLayout(pnlAct, BoxLayout.Y_AXIS));
        btnCalculateOffsets = new JButton(calculateOffsetsAction);
        btnCalculateOffsets.setText("Calculate Offsets from Ref Hole");
        pnlAct.add(btnCalculateOffsets);
        btnSaveToFeeder = new JButton(saveToFeederAction);
        btnSaveToFeeder.setText("Save To Feeder");
        pnlAct.add(btnSaveToFeeder);
        add(pnlAct);
    }

    @Override
    public void createBindings() {
        LengthConverter lc = new LengthConverter();
        IntegerConverter ic = new IntegerConverter();
        DoubleConverter dc = new DoubleConverter(Configuration.get().getLengthDisplayFormat());

        // Tape - partPitch and tapeWidth are inherited from ReferenceStripFeeder
        addWrappedBinding(feeder, "partPitch", textFieldPartPitch, "text", lc);
        addWrappedBinding(feeder, "tapeWidth", textFieldTapeWidth, "text", lc);

        // Slot
        addWrappedBinding(feeder, "row", textFieldRow, "text", ic);
        addWrappedBinding(feeder, "col", textFieldCol, "text", ic);
        addWrappedBinding(feeder, "subType", textFieldSubType, "text", ic);
        // Per-feeder contact offsets + height (flashed by Save To Feeder).
        addWrappedBinding(feeder, "feederContactOffsetX", textFieldContactOX, "text", dc);
        addWrappedBinding(feeder, "feederContactOffsetY", textFieldContactOY, "text", dc);
        addWrappedBinding(feeder, "feederHeight", textFieldFeederHeight, "text", lc);

        // Pick location - inherited from AbstractFeeder via ReferenceFeeder
        MutableLocationProxy pickProxy = new MutableLocationProxy();
        bind(UpdateStrategy.READ_WRITE, feeder, "location", pickProxy, "location");
        addWrappedBinding(pickProxy, "lengthX", textFieldPickX, "text", lc);
        addWrappedBinding(pickProxy, "lengthY", textFieldPickY, "text", lc);
        addWrappedBinding(pickProxy, "lengthZ", textFieldPickZ, "text", lc);

        // Reference hole location - inherited from ReferenceStripFeeder
        MutableLocationProxy refHoleProxy = new MutableLocationProxy();
        bind(UpdateStrategy.READ_WRITE, feeder, "referenceHoleLocation", refHoleProxy, "location");
        addWrappedBinding(refHoleProxy, "lengthX", textFieldRefHoleX, "text", lc);
        addWrappedBinding(refHoleProxy, "lengthY", textFieldRefHoleY, "text", lc);
        addWrappedBinding(refHoleProxy, "lengthZ", textFieldRefHoleZ, "text", lc);

        // Last hole location - inherited from ReferenceStripFeeder
        MutableLocationProxy lastHoleProxy = new MutableLocationProxy();
        bind(UpdateStrategy.READ_WRITE, feeder, "lastHoleLocation", lastHoleProxy, "location");
        addWrappedBinding(lastHoleProxy, "lengthX", textFieldLastHoleX, "text", lc);
        addWrappedBinding(lastHoleProxy, "lengthY", textFieldLastHoleY, "text", lc);
        addWrappedBinding(lastHoleProxy, "lengthZ", textFieldLastHoleZ, "text", lc);

        // Read-only parts-per-hole label
        labelPartsPerHole.setText(Integer.toString(feeder.getPartsPerHole()));

        // Decorate text fields
        ComponentDecorators.decorateWithAutoSelectAndLengthConversion(textFieldPickX);
        ComponentDecorators.decorateWithAutoSelectAndLengthConversion(textFieldPickY);
        ComponentDecorators.decorateWithAutoSelectAndLengthConversion(textFieldPickZ);
        ComponentDecorators.decorateWithAutoSelectAndLengthConversion(textFieldRefHoleX);
        ComponentDecorators.decorateWithAutoSelectAndLengthConversion(textFieldRefHoleY);
        ComponentDecorators.decorateWithAutoSelectAndLengthConversion(textFieldRefHoleZ);
        ComponentDecorators.decorateWithAutoSelectAndLengthConversion(textFieldLastHoleX);
        ComponentDecorators.decorateWithAutoSelectAndLengthConversion(textFieldLastHoleY);
        ComponentDecorators.decorateWithAutoSelectAndLengthConversion(textFieldLastHoleZ);

        ComponentDecorators.decorateWithAutoSelect(textFieldPartPitch);
        ComponentDecorators.decorateWithAutoSelectAndLengthConversion(textFieldTapeWidth);
        ComponentDecorators.decorateWithAutoSelect(textFieldContactOX);
        ComponentDecorators.decorateWithAutoSelect(textFieldContactOY);
        ComponentDecorators.decorateWithAutoSelectAndLengthConversion(textFieldFeederHeight);
    }

    // ================================================================
    // Actions
    // ================================================================

    private Action calculateOffsetsAction = new AbstractAction("CalculateOffsets") {
        {
            putValue(Action.SHORT_DESCRIPTION,
                    "<html>Calculates OX, OY (contact offsets) and Height from the captured<br/>"
                    + "Reference Hole location, then derives the Pick and Last hole positions<br/>"
                    + "from the current Orientation and Tape Width.<br/>"
                    + "Workflow: Discover → jog to ref hole → Capture Ref → set Orientation/<br/>"
                    + "Tape Width → press this → Save To Feeder.</html>");
        }

        @Override
        public void actionPerformed(ActionEvent e) {
            UiUtils.messageBoxOnException(() -> {
                // Push any pending UI edits (ref hole capture, orientation, tape
                // width) into the feeder model before reading them.
                applyAction.actionPerformed(e);
                feeder.calculateOffsetsFromRefHole();
                // The setters fire property changes, but mirror them to the text
                // fields explicitly (matching the CassetteFeeder wizard pattern)
                // so the UI is guaranteed to refresh.
                SwingUtilities.invokeLater(() -> {
                    LengthConverter lc = new LengthConverter();
                    DoubleConverter dc =
                            new DoubleConverter(Configuration.get().getLengthDisplayFormat());
                    textFieldContactOX.setText(dc.convertForward(feeder.getFeederContactOffsetX()));
                    textFieldContactOY.setText(dc.convertForward(feeder.getFeederContactOffsetY()));
                    textFieldFeederHeight.setText(lc.convertForward(feeder.getFeederHeight()));
                    textFieldPickX.setText(lc.convertForward(feeder.getLocation().getLengthX()));
                    textFieldPickY.setText(lc.convertForward(feeder.getLocation().getLengthY()));
                    textFieldPickZ.setText(lc.convertForward(feeder.getLocation().getLengthZ()));
                    textFieldLastHoleX.setText(
                            lc.convertForward(feeder.getLastHoleLocation().getLengthX()));
                    textFieldLastHoleY.setText(
                            lc.convertForward(feeder.getLastHoleLocation().getLengthY()));
                    textFieldLastHoleZ.setText(
                            lc.convertForward(feeder.getLastHoleLocation().getLengthZ()));
                });
            });
        }
    };

    private Action saveToFeederAction = new AbstractAction("Save To Feeder") {
        @Override
        public void actionPerformed(ActionEvent e) {
            // Flush UI values into the feeder model so OX/OY/H/part reflect the
            // latest (possibly just-calculated or hand-edited) state.
            applyAction.actionPerformed(e);
            UiUtils.submitUiMachineTask(() -> {
                Actuator act = CassetteFeederConfigurator.configureActuator();
                CassetteFeederConfigurator cfg = feeder.findConfigurator();
                int pi = (int) Math.round(feeder.getPartPitch()
                        .convertToUnits(LengthUnit.Millimeters).getValue() * 10);
                String cmd = String.format("R%d C%d TC%d TR%d PI%d ST%d OX%d OY%d H%d N%s;",
                        feeder.getRow(), feeder.getCol(),
                        cfg.getTotalCol(), cfg.getTotalRow(), pi,
                        feeder.getSubType(),
                        (int) Math.round(feeder.getFeederContactOffsetX() * 10),
                        (int) Math.round(feeder.getFeederContactOffsetY() * 10),
                        (int) Math.round(feeder.getFeederHeight().getValue() * 10),
                        feeder.getPart() == null ? feeder.getName() : feeder.getPart().getId());
                String resp = act.read(cmd);
                Logger.info("Save to feeder response: {}", resp);
            });
        }
    };
}
