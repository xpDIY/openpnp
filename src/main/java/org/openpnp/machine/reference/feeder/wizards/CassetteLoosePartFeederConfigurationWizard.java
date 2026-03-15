package org.openpnp.machine.reference.feeder.wizards;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.image.BufferedImage;
import java.io.IOException;
import java.lang.reflect.InvocationHandler;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;

import javax.swing.JButton;
import javax.swing.JDialog;
import javax.swing.JPanel;
import javax.swing.border.TitledBorder;

import org.openpnp.gui.MainFrame;
import org.openpnp.machine.reference.feeder.CassetteLoosePartFeeder;
import org.openpnp.model.Configuration;
import org.openpnp.spi.Actuator;
import org.openpnp.spi.Camera;
import org.openpnp.util.UiUtils;
import org.openpnp.vision.pipeline.CvPipeline;
import org.openpnp.vision.pipeline.ui.CvPipelineEditor;
import org.openpnp.vision.pipeline.ui.CvPipelineEditorDialog;

import com.jgoodies.forms.layout.ColumnSpec;
import com.jgoodies.forms.layout.FormLayout;
import com.jgoodies.forms.layout.FormSpecs;
import com.jgoodies.forms.layout.RowSpec;

public class CassetteLoosePartFeederConfigurationWizard
        extends AbstractReferenceFeederConfigurationWizard {
    private final CassetteLoosePartFeeder feeder;

    public CassetteLoosePartFeederConfigurationWizard(CassetteLoosePartFeeder feeder) {
        super(feeder);
        this.feeder = feeder;

        JPanel panel = new JPanel();
        panel.setBorder(new TitledBorder(null, "Vision", TitledBorder.LEADING, TitledBorder.TOP,
                null, null));
        contentPanel.add(panel);
        panel.setLayout(new FormLayout(
                new ColumnSpec[] {FormSpecs.RELATED_GAP_COLSPEC, FormSpecs.DEFAULT_COLSPEC,
                        FormSpecs.RELATED_GAP_COLSPEC, FormSpecs.DEFAULT_COLSPEC,},
                new RowSpec[] {FormSpecs.RELATED_GAP_ROWSPEC, FormSpecs.DEFAULT_ROWSPEC,}));

        JButton btnEditPipeline = new JButton("Edit Pipeline");
        btnEditPipeline.addActionListener(new ActionListener() {
            public void actionPerformed(ActionEvent e) {
                UiUtils.messageBoxOnException(() -> {
                    editPipeline();
                });
            }
        });
        panel.add(btnEditPipeline, "2, 2");

        JButton btnResetPipeline = new JButton("Reset Pipeline");
        btnResetPipeline.addActionListener(new ActionListener() {
            public void actionPerformed(ActionEvent e) {
                resetPipeline();
            }
        });
        panel.add(btnResetPipeline, "4, 2");
    }

    private void editPipeline() throws Exception {
        if (feeder.getPart() == null) {
            throw new Exception("Feeder "+feeder.getName()+" has no part.");
        }
        CvPipeline pipeline = feeder.getPipeline();
        Camera realCamera = Configuration.get().getMachine().getDefaultHead().getDefaultCamera();
        
        // Turn off top lighting
        Actuator lightActuator = realCamera.getLightActuator();
        if (lightActuator != null) {
            lightActuator.actuate(false);
        }
        
        // Create a camera proxy that prevents any light actuator access during editing
        Camera cameraProxy = (Camera) Proxy.newProxyInstance(
            Camera.class.getClassLoader(),
            new Class<?>[] { Camera.class },
            new InvocationHandler() {
                @Override
                public Object invoke(Object proxy, Method method, Object[] args) throws Throwable {
                    // Intercept getLightActuator() to return null, preventing any light control
                    if (method.getName().equals("getLightActuator")) {
                        return null;
                    }
                    // Intercept actuateLightBeforeCapture() to do nothing
                    if (method.getName().equals("actuateLightBeforeCapture")) {
                        return null;
                    }
                    // Intercept actuateLightAfterCapture() to do nothing
                    if (method.getName().equals("actuateLightAfterCapture")) {
                        return null;
                    }
                    // For all other methods, delegate to the real camera
                    return method.invoke(realCamera, args);
                }
            }
        );
        
        // Set the proxy camera on the pipeline - now light cannot be turned on
        pipeline.setProperty("camera", cameraProxy);
        pipeline.setProperty("feeder", feeder);
        
        CvPipelineEditor editor = new CvPipelineEditor(pipeline);
        JDialog dialog = new CvPipelineEditorDialog(MainFrame.get(), feeder.getPart().getId() + " Pipeline", editor);
        dialog.setVisible(true);
    }

    private void resetPipeline() {
        feeder.resetPipeline();
    }
}
