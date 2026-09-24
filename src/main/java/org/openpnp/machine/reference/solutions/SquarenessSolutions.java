/*
 * Copyright (C) 2024 <dev@openpnp.org>
 * inspired and based on work
 * Copyright (C) 2011 Jason von Nieda <jason@vonnieda.org>
 * 
 * This file is part of OpenPnP.
 * 
 * OpenPnP is free software: you can redistribute it and/or modify it under the terms of the GNU
 * General Public License as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
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

package org.openpnp.machine.reference.solutions;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.FlowLayout;
import java.awt.Font;
import java.awt.FontMetrics;
import java.awt.Graphics2D;
import java.awt.GridLayout;
import java.awt.RenderingHints;
import java.awt.Toolkit;
import java.awt.event.ActionEvent;
import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ExecutionException;

import javax.swing.AbstractAction;
import javax.swing.Action;
import javax.swing.ImageIcon;
import javax.swing.JButton;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.SwingConstants;

import org.openpnp.gui.MainFrame;
import org.openpnp.gui.support.Icons;
import org.openpnp.machine.reference.ReferenceHead;
import org.openpnp.machine.reference.ReferenceMachine;
import org.openpnp.machine.reference.axis.ReferenceControllerAxis;
import org.openpnp.machine.reference.axis.ReferenceLinearTransformAxis;
import org.openpnp.machine.reference.camera.ReferenceCamera;
import org.openpnp.model.AxesLocation;
import org.openpnp.model.Length;
import org.openpnp.model.LengthUnit;
import org.openpnp.model.Location;
import org.openpnp.model.Solutions;
import org.openpnp.model.Solutions.Milestone;
import org.openpnp.model.Solutions.Severity;
import org.openpnp.model.Solutions.State;
import org.openpnp.spi.Axis;
import org.openpnp.spi.Axis.Type;
import org.openpnp.spi.Camera;
import org.openpnp.spi.Head;
import org.openpnp.spi.HeadMountable;
import org.openpnp.spi.MotionPlanner.CompletionType;
import org.openpnp.spi.base.AbstractHeadMountable;
import org.openpnp.spi.base.AbstractAxis;
import org.openpnp.spi.base.AbstractHead.VisualHomingMethod;
import org.openpnp.util.MovableUtils;
import org.openpnp.util.UiUtils;
import org.openpnp.vision.pipeline.CvStage.Result.Circle;
import org.openpnp.vision.pipeline.stages.DetectCircularSymmetry.ScoreRange;
import org.pmw.tinylog.Logger;
import org.simpleframework.xml.Attribute;
import org.simpleframework.xml.Element;
import org.simpleframework.xml.ElementList;

/**
 * This helper class implements the Issues &amp; Solutions for calibrating the machine step size
 * and squareness (non-squareness of the X/Y axes), using a regular grid of holes in the machine
 * base plate as the reference.
 * 
 * The idea is that a base plate with a regular grid of holes is a physical reference of an
 * orthogonal, evenly spaced grid. By moving the down-looking camera to a number of these holes
 * and observing where they actually are, we can:
 * <ul>
 * <li>Measure how much the machine's commanded step size deviates from the true hole pitch
 * (step scaling).</li>
 * <li>Measure how much the machine's X and Y axes deviate from a true 90° angle (squareness).</li>
 * </ul>
 * 
 * The calibration scans the whole grid of possible hole locations row by row, collecting every
 * hole that can be detected (occupied/covered holes are skipped). From all those holes the X/Y
 * step scale and the squareness (shear) are determined in a single fit, so a single Issues &amp;
 * Solutions item covers both.
 * 
 * The corrections are applied by creating (or updating) {@link ReferenceLinearTransformAxis}
 * compensation axes for the X and Y axes, so that the user-facing coordinate system becomes
 * correctly scaled and orthogonal.
 */
public class SquarenessSolutions implements Solutions.Subject {

    /**
     * The diameter of the base plate holes. This is only used as a <strong>draft</strong> pixel
     * scale for centering the camera on the holes (together with the actually detected pixel
     * diameter). The step size and squareness are calibrated from the hole pitch.
     */
    @Attribute(required = false)
    private double holeDiameterMm = 3.0;

    /**
     * The base plate hole diameter in camera pixels, as tuned by the user (or auto-detected).
     * Together with {@link #holeDiameterMm} this gives the pixel scale used to locate the hole
     * centers, independent of the camera's stored units per pixel.
     */
    @Attribute(required = false)
    private int holeDiameterPx = 0;

    @Attribute(required = false)
    private double holeSpacingMm = 32.0;

    /**
     * The number of hole columns to scan, starting at the reference hole and going in +X. The
     * reference hole is column 1, so the highest hole index i is {@code holeColumns - 1}.
     */
    @Attribute(required = false)
    private int holeColumns = 10;

    /**
     * The number of hole rows to scan, starting at the reference hole row and going in +Y. The
     * reference hole is row 1, so the highest hole index j is {@code holeRows - 1}.
     */
    @Attribute(required = false)
    private int holeRows = 5;

    // Deprecated pre "Columns/Rows" settings, kept so that older machine.xml files still load.
    @Deprecated
    @Attribute(required = false)
    private int holeSpanX = 0;
    @Deprecated
    @Attribute(required = false)
    private int holeSpanY = 0;

    /**
     * Set when the step size has been calibrated. The squareness calibration is only offered
     * afterwards, as it builds on correctly scaled axes.
     */
    @Attribute(required = false)
    private boolean stepSizeCalibrated = false;

    @Attribute(required = false)
    private boolean squarenessCalibrated = false;

    /**
     * The raw hole measurements of the last calibration, persisted so that the fit can be redone
     * and the heatmap shown again after a restart.
     */
    @Element(required = false)
    private GridCalibrationData lastCalibration;

    private ReferenceMachine machine;

    // The last calibration results, only used for reporting in the UI.
    private Compensation lastStepCompensation;
    private Compensation lastMetricCompensation;
    private int lastHoleCount;
    private List<GridMeasurement> lastMeasurements;
    private double lastSpacing = 0;

    public SquarenessSolutions setMachine(ReferenceMachine machine) {
        this.machine = machine;
        return this;
    }

    public double getHoleDiameterMm() {
        return holeDiameterMm;
    }

    public void setHoleDiameterMm(double holeDiameterMm) {
        this.holeDiameterMm = holeDiameterMm;
    }

    public double getHoleSpacingMm() {
        return holeSpacingMm;
    }

    public void setHoleSpacingMm(double holeSpacingMm) {
        this.holeSpacingMm = holeSpacingMm;
    }

    public int getHoleColumns() {
        if (holeColumns > 0) {
            return holeColumns;
        }
        // Migrate an old symmetric X span (reference at the plate edge): span + 1 columns.
        return holeSpanX > 0 ? holeSpanX + 1 : 10;
    }

    public void setHoleColumns(int holeColumns) {
        this.holeColumns = Math.max(1, holeColumns);
        this.holeSpanX = 0;
    }

    public int getHoleRows() {
        if (holeRows > 0) {
            return holeRows;
        }
        // Migrate an old Y span (rows above the reference): span + 1 rows.
        return holeSpanY > 0 ? holeSpanY + 1 : 5;
    }

    public void setHoleRows(int holeRows) {
        this.holeRows = Math.max(1, holeRows);
        this.holeSpanY = 0;
    }

    public boolean isStepSizeCalibrated() {
        return stepSizeCalibrated;
    }

    public boolean isSquarenessCalibrated() {
        return squarenessCalibrated;
    }

    /**
     * A single measurement of a base plate hole. The grid indices (i, j) are the integer hole
     * offsets relative to the reference (0, 0) hole. The raw coordinates are the untransformed
     * controller axis coordinates in millimeters.
     */
    public static class GridMeasurement {
        public final int i;
        public final int j;
        public final double rawX;
        public final double rawY;

        public GridMeasurement(int i, int j, double rawX, double rawY) {
            this.i = i;
            this.j = j;
            this.rawX = rawX;
            this.rawY = rawY;
        }
    }

    /**
     * The persisted raw measurements of the last base plate grid calibration. Storing these in the
     * machine configuration lets the user redo the fit or show the heatmap again after a restart,
     * without having to re-scan the whole base plate.
     */
    public static class GridCalibrationData {
        @Attribute(required = false)
        private double spacingMm = 32.0;
        @Attribute(required = false)
        private String headName = "";
        @ElementList(required = false, inline = true, entry = "hole")
        private List<Hole> holes = new ArrayList<>();

        public GridCalibrationData() {
        }

        public GridCalibrationData(double spacingMm, String headName,
                List<GridMeasurement> measurements) {
            this.spacingMm = spacingMm;
            this.headName = headName == null ? "" : headName;
            this.holes = new ArrayList<>();
            for (GridMeasurement m : measurements) {
                holes.add(new Hole(m.i, m.j, m.rawX, m.rawY));
            }
        }

        public double getSpacingMm() {
            return spacingMm;
        }

        public String getHeadName() {
            return headName;
        }

        public boolean isEmpty() {
            return holes == null || holes.isEmpty();
        }

        public List<GridMeasurement> toMeasurements() {
            List<GridMeasurement> list = new ArrayList<>();
            if (holes != null) {
                for (Hole h : holes) {
                    list.add(new GridMeasurement(h.i, h.j, h.rawX, h.rawY));
                }
            }
            return list;
        }

        /** One measured hole, in raw axis coordinates. */
        public static class Hole {
            @Attribute
            private int i;
            @Attribute
            private int j;
            @Attribute
            private double rawX;
            @Attribute
            private double rawY;

            public Hole() {
            }

            public Hole(int i, int j, double rawX, double rawY) {
                this.i = i;
                this.j = j;
                this.rawX = rawX;
                this.rawY = rawY;
            }
        }
    }

    /**
     * The result of the step size and squareness calibration. The corrected (transformed)
     * coordinates are computed from the raw coordinates as:
     * 
     * <pre>
     * x' = l11 * x + l12 * y
     * y' =            l22 * y
     * </pre>
     */
    public static class Compensation {
        public final double l11;
        public final double l12;
        public final double l21;
        public final double l22;

        public Compensation(double l11, double l12, double l22) {
            this(l11, l12, 0.0, l22);
        }

        public Compensation(double l11, double l12, double l21, double l22) {
            this.l11 = l11;
            this.l12 = l12;
            this.l21 = l21;
            this.l22 = l22;
        }

        /** The X step size scaling factor. */
        public double getStepScaleX() {
            return Math.hypot(l11, l21);
        }

        /** The Y step size scaling factor. */
        public double getStepScaleY() {
            return Math.hypot(l12, l22);
        }

        /** The non-squareness (shear) factor. */
        public double getSquarenessFactor() {
            return l12;
        }

        /** The deviation from a square angle, in degrees. */
        public double getSquarenessErrorDegrees() {
            // The raw Y axis direction in corrected coordinates is (l12, l22).
            // Ideally it would be (0, l22), i.e. at 90 degrees to the X axis.
            return Math.toDegrees(Math.atan2(l12, l22));
        }

        @Override
        public String toString() {
            return String.format("[%.6f %.6f; %.6f %.6f] step X %.6f, step Y %.6f, squareness %.6f (%.3f°)",
                    l11, l12, l21, l22, getStepScaleX(), getStepScaleY(), l12,
                    getSquarenessErrorDegrees());
        }
    }

    @Override
    public void findIssues(Solutions solutions) {
        if (!solutions.isTargeting(Milestone.Calibration)) {
            return;
        }
        for (Head h : machine.getHeads()) {
            if (!(h instanceof ReferenceHead)) {
                continue;
            }
            ReferenceHead head = (ReferenceHead) h;
            ReferenceCamera camera;
            try {
                camera = (ReferenceCamera) head.getDefaultCamera();
            }
            catch (Exception e) {
                continue;
            }
            if (camera == null || camera.getLooking() != Camera.Looking.Down) {
                continue;
            }
            AbstractAxis rawX = (AbstractAxis) HeadSolutions.getRawAxis(machine, camera.getAxisX());
            AbstractAxis rawY = (AbstractAxis) HeadSolutions.getRawAxis(machine, camera.getAxisY());
            if (!(rawX instanceof ReferenceControllerAxis)
                    || !(rawY instanceof ReferenceControllerAxis)) {
                // We can only calibrate machines with real controller X and Y axes.
                continue;
            }
            final ReferenceCamera finalCamera = camera;

            // Step 1: calibrate the step size first. This makes the axes move a true millimeter
            // before the squareness (shear) is determined.
            solutions.add(new GridIssue(
                    head, 
                    "Step size & squareness: calibrate the X/Y step size and squareness using the base plate hole grid on "+head.getName()+".", 
                    "Move the camera over the base plate holes ("
                            +format(holeSpacingMm)+"mm pitch) to measure and correct the X/Y step size and squareness.", 
                    Severity.Suggestion,
                    "https://github.com/openpnp/openpnp/wiki/Calibration-Solutions#base-plate-hole-grid-step-size-calibration",
                    finalCamera) {

                private Compensation undoCompensation;

                @Override
                public String getExtendedDescription() {
                    StringBuilder str = new StringBuilder();
                    str.append("<html>");
                    str.append("<p>Your machine base plate has a regular grid of holes with a "
                            + "<strong>"+format(holeSpacingMm)+"&nbsp;mm</strong> pitch. The known "
                            + "pitch is used to calibrate the X and Y <strong>step size</strong>.</p>");
                    str.append("<p>First adjust the <strong>Hole diameter [px]</strong> setting on the "
                            + "side (or press <strong>Auto-Detect Hole</strong>) until the green circle "
                            + "in the camera view hugs the hole. The known hole diameter and that "
                            + "pixel diameter are then used to locate each hole center.</p>");
                    str.append("<p>The camera "+finalCamera.getName()+" will scan the whole "
                            + "base plate grid, row by row, and measure every hole it can detect. "
                            + "From all those holes the <strong>average X and Y step size</strong> "
                            + "and the <strong>squareness</strong> (non-perpendicularity of the X/Y "
                            + "axes) are computed using the known "+format(holeSpacingMm)
                            + "&nbsp;mm pitch.</p>");
                    str.append("<p>Holes that are occupied or covered and therefore cannot be "
                            + "detected are automatically skipped, and the scan continues with the "
                            + "next hole. A heatmap of the local step size variation across the base "
                            + "plate is shown at the end.</p>");
                    str.append("<p>Make sure the machine is homed. Jog the down-looking camera "
                            + finalCamera.getName()+" approximately over the bottom-left reference "
                            + "hole - the scan only goes right (+X) and up (+Y) from there - then "
                            + "press <strong>Accept</strong>.</p>");
                    str.append("<p><strong color=\"red\">CAUTION</strong>: The camera "
                            + finalCamera.getName()+" will move across the base plate, visiting up to "
                            + (getHoleColumns()*getHoleRows())+" holes. Make sure the path is clear "
                            + "of obstacles and that the camera is focused on the holes.</p>");
                    str.append("<p><strong>Note:</strong> changing the transform shifts the machine "
                            + "coordinate system. The machine will perform a visual homing cycle if "
                            + "visual homing is configured. Otherwise, re-home the machine, and "
                            + "revisit previously captured locations (fiducials, feeders, boards) that "
                            + "must stay physically in place.</p>");
                    if (getState() == State.Solved && lastStepCompensation != null) {
                        str.append("<br/><h4>Results ("+lastHoleCount+" holes):</h4>");
                        str.append("<table>");
                        str.append("<tr><td align=\"right\">X step scale:</td><td>"
                                +String.format("%.6f", lastStepCompensation.l11)+"</td></tr>");
                        str.append("<tr><td align=\"right\">Y step scale:</td><td>"
                                +String.format("%.6f", lastStepCompensation.l22)+"</td></tr>");
                        str.append("<tr><td align=\"right\">Actual X movement per "+format(holeSpacingMm)+"&nbsp;mm:</td><td>"
                                +String.format("%.3f", holeSpacingMm * lastStepCompensation.l11)+"&nbsp;mm</td></tr>");
                        str.append("<tr><td align=\"right\">Actual Y movement per "+format(holeSpacingMm)+"&nbsp;mm:</td><td>"
                                +String.format("%.3f", holeSpacingMm * lastStepCompensation.l22)+"&nbsp;mm</td></tr>");
                        str.append("<tr><td align=\"right\">Non-squareness factor (X&lt;-Y):</td><td>"
                                +String.format("%.6f", lastStepCompensation.l12)+"</td></tr>");
                        str.append("<tr><td align=\"right\">Non-squareness factor (Y&lt;-X):</td><td>"
                                +String.format("%.6f", lastStepCompensation.l21)+"</td></tr>");
                        str.append("<tr><td align=\"right\">Squareness error:</td><td>"
                                +String.format("%.4f", lastStepCompensation.getSquarenessErrorDegrees())+"°</td></tr>");
                        str.append("</table>");
                        str.append("<p>The step size and squareness are applied together as software "
                                + "transform axes. If your controller firmware has a steps/mm setting, "
                                + "you may instead correct it there by multiplying the X steps/mm by "
                                + String.format("%.4f", 1.0/lastStepCompensation.l11)+" and the Y steps/mm by "
                                + String.format("%.4f", 1.0/lastStepCompensation.l22)
                                +", then re-run this calibration.</p>");
                    }
                    str.append("</html>");
                    return str.toString();
                }

                @Override
                public void setState(Solutions.State state) throws Exception {
                    if (state == State.Solved) {
                        validateCalibrationPreconditions(finalCamera);
                        undoCompensation = getCurrentCompensation(finalCamera);
                        final State oldState = getState();
                        UiUtils.submitUiMachineTask(
                                () -> {
                                    calibrateStepSize(head, finalCamera);
                                    return true;
                                },
                                (result) -> {
                                    UiUtils.messageBoxOnException(() -> super.setState(state));
                                    solutions.setSolutionsIssueSolved(this, true);
                                    MainFrame.get().getIssuesAndSolutionsTab().findIssuesAndSolutions();
                                },
                                (t) -> {
                                    UiUtils.showError(t);
                                    UiUtils.messageBoxOnException(() -> setState(oldState));
                                });
                    }
                    else {
                        restoreCompensation(finalCamera, undoCompensation);
                        stepSizeCalibrated = false;
                        squarenessCalibrated = false;
                        solutions.setSolutionsIssueSolved(this, false);
                        super.setState(state);
                    }
                }
            });
        }
    }

    /**
     * Common base class for the base plate hole grid issue. It provides the shared activation
     * behavior and the hole grid settings.
     */
    private abstract class GridIssue extends Solutions.Issue {
        protected final ReferenceCamera camera;

        protected GridIssue(Solutions.Subject subject, String issue, String solution, Severity severity,
                String uri, ReferenceCamera camera) {
            super(subject, issue, solution, severity, uri);
            this.camera = camera;
        }

        @Override
        public void activate() throws Exception {
            MainFrame.get().getMachineControls().setSelectedTool(camera);
            camera.ensureCameraVisible();
        }

        @Override
        public Solutions.Issue.CustomProperty[] getProperties() {
            return new Solutions.Issue.CustomProperty[] {
                    new Solutions.Issue.DoubleProperty(
                            "Hole diameter [mm]",
                            "The diameter of the base plate holes. Used as a draft scale for centering the camera on the holes. The step size and squareness are calibrated from the hole pitch.",
                            0.1, 50.0) {
                        @Override
                        public double get() {
                            return holeDiameterMm;
                        }

                        @Override
                        public void set(double value) {
                            holeDiameterMm = value;
                        }
                    },
                    new Solutions.Issue.IntegerProperty(
                            "Hole diameter [px]",
                            "The base plate hole diameter in camera pixels. Adjust it up/down until the green circle in the camera view hugs the hole. Together with the known hole diameter this gives the pixel scale used to locate the hole centers.",
                            3, Math.max(100, (int)(Math.min(camera.getWidth(), camera.getHeight())*0.8))) {
                        @Override
                        public int get() {
                            return holeDiameterPx;
                        }

                        @Override
                        public void set(int value) {
                            holeDiameterPx = value;
                            previewHole(camera, value);
                        }
                    },
                    new Solutions.Issue.ActionProperty(
                            "",
                            "Automatically detect the base plate hole diameter in pixels.") {
                        @Override
                        public Action get() {
                            return new AbstractAction("Auto-Detect Hole", Icons.rotateCounterclockwise) {
                                @Override
                                public void actionPerformed(ActionEvent e) {
                                    UiUtils.submitUiMachineTask(() -> {
                                        holeDiameterPx = autoDetectHoleDiameterPx(camera);
                                        showHolePreview(camera, holeDiameterPx);
                                        return true;
                                    });
                                }
                            };
                        }
                    },
                    new Solutions.Issue.DoubleProperty(
                            "Hole pitch [mm]",
                            "The regular distance between adjacent base plate holes. Used to calibrate the axis step size.",
                            0.1, 500.0) {
                        @Override
                        public double get() {
                            return holeSpacingMm;
                        }

                        @Override
                        public void set(double value) {
                            holeSpacingMm = value;
                        }
                    },
                    new Solutions.Issue.IntegerProperty(
                            "Columns",
                            "The number of hole columns to scan, starting at the reference hole and "
                            + "going in +X. The reference hole is column 1.",
                            1, 30) {
                        @Override
                        public int get() {
                            return getHoleColumns();
                        }

                        @Override
                        public void set(int value) {
                            setHoleColumns(value);
                        }
                    },
                    new Solutions.Issue.IntegerProperty(
                            "Rows",
                            "The number of hole rows to scan, starting at the reference hole row and "
                            + "going in +Y. The reference hole is row 1.",
                            1, 30) {
                        @Override
                        public int get() {
                            return getHoleRows();
                        }

                        @Override
                        public void set(int value) {
                            setHoleRows(value);
                        }
                    },
                    new Solutions.Issue.ComponentProperty(
                            "Heatmap",
                            "The local step size variation across the base plate, from the last scan.") {
                        @Override
                        public javax.swing.JComponent get() {
                            return createHeatmapComponent(camera);
                        }
                    },
            };
        }
    }

    private void validateCalibrationPreconditions(ReferenceCamera camera) throws Exception {
        if (!machine.isEnabled() || !machine.isHomed()) {
            throw new Exception("The machine must be enabled and homed for the calibration.");
        }
        if (!isCameraCalibrated(camera)) {
            throw new Exception("The position of camera "
                    +camera.getName()+" must be initialized first.");
        }
    }

    /**
     * Perform the step size (scale) calibration. This must run on the machine thread.
     * 
     * @param head
     * @param camera
     * @return the measured compensation
     * @throws Exception
     */
    public Compensation calibrateStepSize(ReferenceHead head, ReferenceCamera camera) throws Exception {
        validateCalibrationPreconditions(camera);
        List<GridMeasurement> measurements = measureGrid(head, camera);
        Compensation metric = fitCompensation(measurements, holeSpacingMm);
        // The whole-plate scan determines the X/Y scale and the squareness (shear) at the same
        // time. Use the full 2x2 affine fit so that both shear terms are corrected.
        Compensation compensation = fitAffine(measurements, holeSpacingMm);
        Logger.info("Base plate grid fit: metric {}, affine {}", metric, compensation);
        // The affine fit also determines the rotation of the base plate grid relative to the raw
        // machine axes. Applying that would rotate the whole user coordinate system, so only do it
        // if the grid is reasonably aligned with the machine (the normal case for the machine's own
        // base plate). Otherwise fall back to the scale/shear-only correction.
        double gridRotationDegrees = Math.toDegrees(Math.atan2(compensation.l21, compensation.l11));
        if (Math.abs(gridRotationDegrees) > 5.0) {
            Logger.warn("Base plate grid is rotated by {}° relative to the machine axes. Applying "
                    + "the scale/shear-only correction to avoid rotating the coordinate system.",
                    String.format("%.2f", gridRotationDegrees));
            compensation = metric;
        }
        else {
            Logger.info("Base plate grid rotation relative to the machine axes: {}°.",
                    String.format("%.3f", gridRotationDegrees));
        }
        applyCompensation(camera, compensation);

        // Because this change affects the coordinate system, perform a (visual) homing cycle to
        // re-reference the machine coordinates.
        if (head.getVisualHomingMethod() == VisualHomingMethod.ResetToFiducialLocation) {
            head.visualHome(machine, true);
        }

        lastStepCompensation = compensation;
        lastMetricCompensation = metric;
        lastHoleCount = measurements.size();
        lastMeasurements = measurements;
        lastSpacing = holeSpacingMm;
        // Persist the raw measurements so the fit can be redone and the heatmap shown later.
        lastCalibration = new GridCalibrationData(holeSpacingMm, head.getName(), measurements);
        stepSizeCalibrated = true;
        squarenessCalibrated = true;
        Logger.info("Step size and squareness calibration of {}: X {} Y {} squareness {} ({} holes)",
                head.getName(), compensation.l11, compensation.l22, compensation.l12,
                measurements.size());
        return compensation;
    }

    /**
     * Move the camera to the reference hole, then scan the whole base plate grid row by row,
     * starting at the reference (bottom) row and going up to larger Y, collecting the raw
     * coordinates of every hole that can be detected. Occupied, covered or otherwise undetectable
     * holes are skipped.
     */
    private List<GridMeasurement> measureGrid(ReferenceHead head, ReferenceCamera camera) throws Exception {
        AbstractAxis rawX = (AbstractAxis) HeadSolutions.getRawAxis(machine, camera.getAxisX());
        AbstractAxis rawY = (AbstractAxis) HeadSolutions.getRawAxis(machine, camera.getAxisY());
        if (!(rawX instanceof ReferenceControllerAxis)
                || !(rawY instanceof ReferenceControllerAxis)) {
            throw new Exception("The camera "+camera.getName()
            +" must be mapped to two controller X and Y axes for the calibration.");
        }
        Length holeDiameter = new Length(holeDiameterMm, LengthUnit.Millimeters);
        double spacing = holeSpacingMm;

        // Center on the base plate hole that the user has jogged to.
        Location reference = camera.getLocation();
        MovableUtils.moveToLocationAtSafeZ(camera, reference);
        reference = centerOnHole(camera, holeDiameter, "Base plate grid: reference hole",
                0.3, 0.45);
        return scanGrid(camera, rawX, rawY, holeDiameter, spacing, reference);
    }

    /**
     * Scan the grid of base plate hole locations, row by row from the reference row upward. The
     * reference hole is assumed to be the bottom-left hole, so the scan goes in +X (right) and +Y
     * (up) only. Each row is anchored at the reference column and then walked to the right. Every
     * hole that can be detected (using the same circle detection as the rest of the calibration) is
     * centered on and recorded. Holes that cannot be detected are skipped.
     *
     * @return the raw-coordinate measurements of all detected holes
     */
    private List<GridMeasurement> scanGrid(ReferenceCamera camera, AbstractAxis rawX,
            AbstractAxis rawY, Length holeDiameter, double spacing, Location reference)
                    throws Exception {
        final int nx = getHoleColumns() - 1;
        final int ny = getHoleRows() - 1;
        List<GridMeasurement> measurements = new ArrayList<>();
        Map<Long, Location> holes = new HashMap<>();
        holes.put(gridKey(0, 0), reference);
        measurements.add(measureHole(rawX, rawY, camera, reference, 0, 0));

        // Bootstrap the two step vectors (transformed millimeters per pitch).
        Location stepX = new Location(LengthUnit.Millimeters, spacing, 0, 0, 0);
        Location stepY = new Location(LengthUnit.Millimeters, 0, spacing, 0, 0);
        Location hole10 = tryCenterAt(reference.add(stepX), camera, holeDiameter, "(1,0)", 0.3, 0.3);
        if (hole10 != null) {
            stepX = hole10.subtract(reference);
            holes.put(gridKey(1, 0), hole10);
            measurements.add(measureHole(rawX, rawY, camera, hole10, 1, 0));
        }
        Location hole01 = tryCenterAt(reference.add(stepY), camera, holeDiameter, "(0,1)", 0.3, 0.3);
        if (hole01 != null) {
            stepY = hole01.subtract(reference);
            holes.put(gridKey(0, 1), hole01);
            measurements.add(measureHole(rawX, rawY, camera, hole01, 0, 1));
        }
        if (hole10 == null && hole01 == null) {
            throw new Exception("Could not detect any hole next to the reference hole. "
                    + "Make sure the hole diameter/pitch settings are correct.");
        }

        // Scan row by row, from the reference (bottom) row upward to larger Y.
        for (int j = 0; j <= ny; j++) {
            // Anchor the row at the center column i=0.
            if (!holes.containsKey(gridKey(0, j))) {
                Location prev0 = holes.get(gridKey(0, j - 1));
                Location predicted = (prev0 != null)
                        ? prev0.add(stepY)
                        : reference.add(stepY.multiply(j));
                Location anchor = tryCenterAt(predicted, camera, holeDiameter, "(0,"+j+")", 0.1, 0.2);
                if (anchor != null) {
                    if (prev0 != null) {
                        stepY = anchor.subtract(prev0);
                    }
                    holes.put(gridKey(0, j), anchor);
                    measurements.add(measureHole(rawX, rawY, camera, anchor, 0, j));
                }
            }
            Location anchor = holes.get(gridKey(0, j));
            // Walk to the right from the anchor (the reference is the left-most column).
            Location cursor = anchor;
            Location lastFound = anchor;
            int lastFoundI = 0;
            if (cursor == null) {
                cursor = reference.add(stepY.multiply(j));
                lastFound = cursor;
            }
            for (int i = 1; i <= nx; i++) {
                Long key = gridKey(i, j);
                if (holes.containsKey(key)) {
                    cursor = holes.get(key);
                    lastFound = cursor;
                    lastFoundI = i;
                    continue;
                }
                Location predicted = cursor.add(stepX);
                Location centered = tryCenterAt(predicted, camera, holeDiameter,
                        "("+i+","+j+")", 0.1, 0.2);
                if (centered != null) {
                    if (i != lastFoundI) {
                        stepX = centered.subtract(lastFound).multiply(1.0 / (i - lastFoundI));
                    }
                    lastFound = centered;
                    lastFoundI = i;
                    cursor = centered;
                    holes.put(key, centered);
                    measurements.add(measureHole(rawX, rawY, camera, centered, i, j));
                }
                else {
                    // Continue along the nominal grid direction.
                    cursor = predicted;
                }
            }
        }
        if (measurements.size() < 3) {
            throw new Exception("Not enough base plate holes could be detected for the calibration "
                    +"(found "+measurements.size()+"). "
                    +"Make sure the camera is focused on the hole grid, the hole diameter/pitch "
                    +"settings are correct, and that the grid is not completely covered.");
        }
        Logger.info("Base plate grid scan: {} of {} possible holes detected.",
                measurements.size(), (nx+1)*(ny+1));
        return measurements;
    }

    /**
     * Move to the predicted location and try to center on a hole there. Returns null (and skips)
     * if the hole cannot be detected.
     */
    private Location tryCenterAt(Location predicted, ReferenceCamera camera, Length holeDiameter,
            String label, double searchRange, double maxOffsetRatio) {
        try {
            camera.moveTo(predicted);
            camera.waitForCompletion(CompletionType.WaitForStillstand);
            return centerOnHole(camera, holeDiameter, "Base plate grid: hole "+label,
                    searchRange, maxOffsetRatio);
        }
        catch (Exception e) {
            Logger.info("Base plate grid: hole {} not found at {} ({}), skipping.",
                    label, predicted, e.getMessage());
            return null;
        }
    }

    private static long gridKey(int i, int j) {
        return (((long) i) << 32) ^ (j & 0xffffffffL);
    }

    /**
     * Build the embedded heatmap component: the local steps-per-unit deviation across the base
     * plate, one map for X and one for Y, side by side. Blue means the local step is smaller than
     * average, red means larger. Returns a hint label if no scan data is available yet.
     */
    private javax.swing.JComponent createHeatmapComponent(ReferenceCamera camera) {
        List<GridMeasurement> measurements = getCalibrationMeasurements();
        Compensation metric = getCalibrationMetric();
        JPanel wrapper = new JPanel(new BorderLayout(0, 4));
        wrapper.setOpaque(false);
        if (measurements == null || measurements.isEmpty() || metric == null) {
            wrapper.add(new JLabel("No scan data yet - press Accept to run the calibration."),
                    BorderLayout.CENTER);
            return wrapper;
        }
        Map<Long, GridMeasurement> byIndex = new HashMap<>();
        for (GridMeasurement m : measurements) {
            byIndex.put(gridKey(m.i, m.j), m);
        }
        // Only render the range of holes that was actually detected, so that unreachable /
        // undetected columns do not widen the map or push the detected holes to one side.
        int minI = Integer.MAX_VALUE;
        int maxI = Integer.MIN_VALUE;
        int minJ = Integer.MAX_VALUE;
        int maxJ = Integer.MIN_VALUE;
        for (GridMeasurement m : measurements) {
            minI = Math.min(minI, m.i);
            maxI = Math.max(maxI, m.i);
            minJ = Math.min(minJ, m.j);
            maxJ = Math.max(maxJ, m.j);
        }
        double spacing = getCalibrationSpacing();
        BufferedImage xImage = createStepSizeHeatmap(byIndex, minI, maxI, minJ, maxJ, spacing,
                metric.l11, true);
        BufferedImage yImage = createStepSizeHeatmap(byIndex, minI, maxI, minJ, maxJ, spacing,
                metric.l22, false);
        JPanel heatmap = new JPanel(new GridLayout(1, 2, 2, 0));
        heatmap.setOpaque(false);
        heatmap.add(createHeatmapColumn(xImage, "X", metric.l11));
        heatmap.add(createHeatmapColumn(yImage, "Y", metric.l22));
        wrapper.add(heatmap, BorderLayout.CENTER);
        JLabel legend = new JLabel("Step scale heatmap - blue < avg < red", SwingConstants.CENTER);
        legend.setFont(legend.getFont().deriveFont(Font.PLAIN, 11f));
        wrapper.add(legend, BorderLayout.NORTH);
        JButton refit = new JButton("Re-fit & apply from stored data");
        refit.setToolTipText("Recompute the step size and squareness from the stored hole "
                + "measurements and apply the correction, without re-scanning the base plate.");
        refit.addActionListener(e -> refitFromStoredData(camera));
        JPanel buttons = new JPanel(new FlowLayout(FlowLayout.LEFT, 0, 0));
        buttons.setOpaque(false);
        buttons.add(refit);
        wrapper.add(buttons, BorderLayout.SOUTH);
        return wrapper;
    }

    private List<GridMeasurement> getCalibrationMeasurements() {
        if (lastMeasurements != null && !lastMeasurements.isEmpty()) {
            return lastMeasurements;
        }
        if (lastCalibration != null && !lastCalibration.isEmpty()) {
            return lastCalibration.toMeasurements();
        }
        return null;
    }

    private double getCalibrationSpacing() {
        if (lastCalibration != null) {
            return lastCalibration.getSpacingMm();
        }
        return lastSpacing > 0 ? lastSpacing : holeSpacingMm;
    }

    private Compensation getCalibrationMetric() {
        if (lastMetricCompensation != null) {
            return lastMetricCompensation;
        }
        List<GridMeasurement> measurements = getCalibrationMeasurements();
        if (measurements != null) {
            try {
                return fitCompensation(measurements, getCalibrationSpacing());
            }
            catch (Exception e) {
                Logger.warn(e);
            }
        }
        return null;
    }

    private void refitFromStoredData(ReferenceCamera camera) {
        List<GridMeasurement> measurements = getCalibrationMeasurements();
        if (measurements == null || measurements.isEmpty() || camera == null) {
            return;
        }
        try {
            double spacing = getCalibrationSpacing();
            Compensation metric = fitCompensation(measurements, spacing);
            Compensation affine = fitAffine(measurements, spacing);
            double gridRotationDegrees = Math.toDegrees(Math.atan2(affine.l21, affine.l11));
            Compensation applied = (Math.abs(gridRotationDegrees) > 5.0) ? metric : affine;
            UiUtils.submitUiMachineTask(() -> {
                applyCompensation(camera, applied);
                return true;
            }).get();
            lastStepCompensation = applied;
            lastMetricCompensation = metric;
            lastMeasurements = measurements;
            lastSpacing = spacing;
            Logger.info("Base plate grid re-fit from stored data: metric {}, affine {} -> applied {}",
                    metric, affine, applied);
        }
        catch (Exception e) {
            UiUtils.showError(e);
        }
    }

    private javax.swing.JComponent createHeatmapColumn(BufferedImage image, String axis,
            double avgScale) {
        JPanel column = new JPanel(new BorderLayout(0, 2));
        column.setOpaque(false);
        JLabel title = new JLabel(axis+"   avg "+String.format("%.6f", avgScale),
                SwingConstants.CENTER);
        title.setFont(title.getFont().deriveFont(Font.PLAIN, 11f));
        column.add(title, BorderLayout.NORTH);
        column.add(new JLabel(new ImageIcon(image)), BorderLayout.CENTER);
        return column;
    }

    private BufferedImage createStepSizeHeatmap(Map<Long, GridMeasurement> byIndex,
            int minI, int maxI, int minJ, int maxJ,
            double spacing, double avgScale, boolean xAxis) {
        final int cell = 36;
        int width = (maxI - minI + 1) * cell;
        int height = (maxJ - minJ + 1) * cell;
        BufferedImage image = new BufferedImage(width, height, BufferedImage.TYPE_INT_RGB);
        Graphics2D g = image.createGraphics();
        g.setRenderingHint(RenderingHints.KEY_TEXT_ANTIALIASING,
                RenderingHints.VALUE_TEXT_ANTIALIAS_ON);
        g.setColor(Color.WHITE);
        g.fillRect(0, 0, width, height);
        // Find the maximum deviation for the color scale.
        double maxDeviation = 1e-9;
        for (GridMeasurement m : byIndex.values()) {
            Double local = localStepScale(byIndex, m, spacing, xAxis);
            if (local != null) {
                maxDeviation = Math.max(maxDeviation, Math.abs(local - avgScale));
            }
        }
        Font cellFont = new Font(Font.SANS_SERIF, Font.PLAIN, 9);
        for (GridMeasurement m : byIndex.values()) {
            if (m.i < minI || m.i > maxI || m.j < minJ || m.j > maxJ) {
                continue;
            }
            int cx = (m.i - minI) * cell;
            int cy = (maxJ - m.j) * cell;
            Double local = localStepScale(byIndex, m, spacing, xAxis);
            Color color = Color.LIGHT_GRAY;
            if (local != null) {
                color = colorForDeviation((local - avgScale) / maxDeviation);
            }
            g.setColor(color);
            g.fillRect(cx, cy, cell - 1, cell - 1);
            g.setColor(Color.BLACK);
            g.drawRect(cx, cy, cell - 1, cell - 1);
            if (local != null) {
                g.setFont(cellFont);
                FontMetrics fm = g.getFontMetrics();
                String text = String.format("%.4f", local);
                int tx = cx + (cell - 1 - fm.stringWidth(text)) / 2;
                int ty = cy + (cell - 1 + fm.getAscent() - fm.getDescent()) / 2;
                g.setColor(Color.BLACK);
                g.drawString(text, tx, ty);
            }
        }
        g.dispose();
        return image;
    }

    /**
     * @return the local step scale (raw pitch / nominal pitch) at the given hole, measured to an
     *         adjacent detected hole, or null if no neighbor was detected.
     */
    private Double localStepScale(Map<Long, GridMeasurement> byIndex, GridMeasurement m,
            double spacing, boolean xAxis) {
        GridMeasurement neighbor = xAxis
                ? byIndex.get(gridKey(m.i + 1, m.j))
                : byIndex.get(gridKey(m.i, m.j + 1));
        if (neighbor == null) {
            neighbor = xAxis
                    ? byIndex.get(gridKey(m.i - 1, m.j))
                    : byIndex.get(gridKey(m.i, m.j - 1));
        }
        if (neighbor == null) {
            return null;
        }
        double dx = neighbor.rawX - m.rawX;
        double dy = neighbor.rawY - m.rawY;
        // The raw step size per nominal pitch; invert it so it is the same kind of quantity as the
        // average step scale (physical movement per raw unit).
        return spacing / Math.hypot(dx, dy);
    }

    private static Color colorForDeviation(double normalized) {
        double v = Math.max(-1.0, Math.min(1.0, normalized));
        if (v < 0) {
            int gb = (int) (255 * (1 + v));
            return new Color(gb, gb, 255);
        }
        else {
            int gb = (int) (255 * (1 - v));
            return new Color(255, gb, gb);
        }
    }

    /**
     * Move the camera onto the center of a base plate hole and verify it. The pixel scale is
     * derived from the known hole diameter and the hole diameter in pixels that the user has tuned
     * (or that was auto-detected), <strong>not</strong> from the camera's stored units per pixel
     * (which may be calibrated at a different Z than the base plate). The camera is re-detected and
     * re-moved until the hole is centered within a pixel tolerance, and the result is verified. If
     * the hole cannot be centered, an exception is thrown (the caller may then skip the hole).
     *
     * @param camera
     * @param holeDiameter
     * @param diagnostics
     * @param searchRange      relative search range around the expected (image center) position
     * @param maxOffsetRatio   maximum allowed pixel offset (relative to the image) of the detection
     * @return the centered (HeadMountable) location
     * @throws Exception if the hole cannot be detected or centered, or the detection is too far
     */
    private Location centerOnHole(ReferenceCamera camera, Length holeDiameter, String diagnostics,
            double searchRange, double maxOffsetRatio) throws Exception {
        VisionSolutions visionSolutions = machine.getVisionSolutions();
        if (holeDiameterPx < 3) {
            // Not tuned yet: auto-detect a starting value.
            holeDiameterPx = autoDetectHoleDiameterPx(camera);
        }
        final int maxPasses = 8;
        final double tolerancePx = 0.75;
        // The pixel scale at the base plate height, from the known hole diameter and the pixel
        // diameter that the user tuned to hug the hole.
        final double mmPerPx = holeDiameter.convertToUnits(LengthUnit.Millimeters).getValue()
                / holeDiameterPx;
        // Reject detections that are too far from the expected hole position. The base plate has
        // extra holes in the center of each 4-hole square, which must not be picked up.
        final double maxOffsetPx = maxOffsetRatio
                * Math.min(camera.getWidth(), camera.getHeight());
        Circle expected = new Circle(0, 0, holeDiameterPx);
        Location location = camera.getLocation();
        for (int pass = 0; pass < maxPasses; pass++) {
            Circle detected = visionSolutions.getSubjectPixelLocation(camera, camera, expected,
                    searchRange, null, null, false);
            double offsetX = detected.x - camera.getWidth() / 2.0;
            double offsetY = camera.getHeight() / 2.0 - detected.y;
            double offsetPx = Math.hypot(offsetX, offsetY);
            if (offsetPx > maxOffsetPx) {
                throw new Exception("The detected feature is too far from the expected hole position ("
                        +String.format("%.1f", offsetPx)+" px). It may be a different hole.");
            }
            if (offsetPx < tolerancePx) {
                break;
            }
            Location offset = new Location(LengthUnit.Millimeters,
                    offsetX * mmPerPx, offsetY * mmPerPx, 0, 0);
            location = camera.getLocation().add(offset);
            // Move directly at the current (plate) Z so the hole stays in focus.
            camera.moveTo(location);
            camera.waitForCompletion(CompletionType.WaitForStillstand);
            // After the first move the hole is near the center, so search tightly from now on.
            searchRange = 0.05;
        }
        // Verify the final centering (also shows a diagnostic image).
        Circle detected = visionSolutions.getSubjectPixelLocation(camera, camera, expected,
                0.05, diagnostics, null, false);
        double residualX = detected.x - camera.getWidth() / 2.0;
        double residualY = camera.getHeight() / 2.0 - detected.y;
        double residualPx = Math.hypot(residualX, residualY);
        double residualMm = residualPx * mmPerPx;
        Logger.info("Base plate hole centering residual: {} px ({} mm), hole diameter {} px",
                String.format("%.2f", residualPx), String.format("%.4f", residualMm), holeDiameterPx);
        if (residualPx > 5.0) {
            throw new Exception("Could not center on the base plate hole (residual "
                    +String.format("%.1f", residualPx)+" px / "
                    +String.format("%.3f", residualMm)+" mm). Check focus and lighting.");
        }
        return camera.getLocation();
    }

    /**
     * Detect the base plate hole diameter in pixels by sweeping a range of diameters and picking the
     * one with the best circular symmetry score. This is done on the machine thread.
     */
    private int autoDetectHoleDiameterPx(ReferenceCamera camera) throws Exception {
        VisionSolutions visionSolutions = machine.getVisionSolutions();
        int maxDiameter = (int)(Math.min(camera.getWidth(), camera.getHeight())*0.8);
        int bestDiameter = 0;
        double bestScore = Double.NEGATIVE_INFINITY;
        for (double diameter = 5; diameter <= maxDiameter; diameter *= 1.1) {
            int d = (int) Math.round(diameter);
            ScoreRange scoreRange = new ScoreRange();
            try {
                visionSolutions.getSubjectPixelLocation(camera, camera, new Circle(0, 0, d),
                        0.4, null, scoreRange, true);
                if (scoreRange.finalScore > bestScore) {
                    bestScore = scoreRange.finalScore;
                    bestDiameter = d;
                }
            }
            catch (Exception e) {
                // Not detectable at this diameter.
            }
        }
        if (bestDiameter < 3) {
            bestDiameter = 20;
        }
        Logger.info("Base plate hole auto-detected diameter: {} px (score {})", bestDiameter, bestScore);
        return bestDiameter;
    }

    /**
     * Show a live detection preview for the given pixel diameter in the camera view (called from the
     * EDT; blocks until the machine task is done).
     */
    private void previewHole(ReferenceCamera camera, int diameterPx) {
        try {
            UiUtils.submitUiMachineTask(() -> {
                showHolePreview(camera, diameterPx);
                return true;
            }).get();
        }
        catch (InterruptedException | ExecutionException e) {
            Logger.warn(e);
        }
    }

    /**
     * Detect the hole with the given pixel diameter and show the result in the camera view. This
     * runs on the machine thread.
     */
    private void showHolePreview(ReferenceCamera camera, int diameterPx) {
        try {
            machine.getVisionSolutions().getSubjectPixelLocation(camera, camera,
                    new Circle(0, 0, diameterPx), 0.4,
                    "Hole diameter "+diameterPx+" px - Score {score}", null, true);
        }
        catch (Exception e) {
            Toolkit.getDefaultToolkit().beep();
        }
    }

    private boolean isCameraCalibrated(ReferenceCamera camera) {
        // Note: the calibration does not require the camera units per pixel (which is calibrated
        // for a different Z plane). It only requires the camera position to be valid, so that the
        // observed moves can be turned into raw axis coordinates.
        try {
            return camera.getLocation().isInitialized();
        }
        catch (Exception e) {
            return false;
        }
    }

    private GridMeasurement measureHole(AbstractAxis rawX, AbstractAxis rawY, HeadMountable hm,
            Location location, int i, int j) throws Exception {
        // The location is a HeadMountable location. Convert to Head, then to raw (untransformed) coordinates.
        Location headLocation = hm.toHeadLocation(location);
        AxesLocation raw = hm.toRaw(headLocation);
        double x = raw.getCoordinate(rawX);
        double y = raw.getCoordinate(rawY);
        Logger.info("Base plate grid: measured hole ({},{}) raw ({}, {}).", i, j,
                String.format("%.4f", x), String.format("%.4f", y));
        return new GridMeasurement(i, j, x, y);
    }

    /**
     * Fit the metric tensor of the physical space as seen in raw axis coordinates from a set of
     * hole measurements. The correction is then the upper triangular Cholesky factor of that
     * metric, which maps the (possibly sheared and scaled) raw coordinates to an orthogonal,
     * correctly scaled coordinate system.
     * 
     * @param measurements
     * @param spacingMm
     * @return
     * @throws Exception
     */
    public static Compensation fitCompensation(List<GridMeasurement> measurements, double spacingMm)
            throws Exception {
        GridMeasurement origin = null;
        for (GridMeasurement m : measurements) {
            if (m.i == 0 && m.j == 0) {
                origin = m;
                break;
            }
        }
        if (origin == null) {
            throw new Exception("The reference (0,0) hole was not measured.");
        }
        // Accumulate the normal equations for the linear system in (g11, g12, g22), the symmetric
        // metric tensor. For any two measurements p, q (relative to the origin), the physical dot
        // product of the two grid displacements is dp . dq:
        //   wp^T G wq = dp . dq
        // where w are the raw displacements and d are the ideal (orthogonal) grid displacements.
        double[][] m = new double[3][3];
        double[] b = new double[3];
        int n = measurements.size();
        int equations = 0;
        for (int a = 0; a < n; a++) {
            for (int c = a; c < n; c++) {
                GridMeasurement pa = measurements.get(a);
                GridMeasurement pb = measurements.get(c);
                if ((pa.i == 0 && pa.j == 0) || (pb.i == 0 && pb.j == 0)) {
                    continue;
                }
                double w1x = pa.rawX - origin.rawX;
                double w1y = pa.rawY - origin.rawY;
                double w2x = pb.rawX - origin.rawX;
                double w2y = pb.rawY - origin.rawY;
                double d1x = pa.i * spacingMm;
                double d1y = pa.j * spacingMm;
                double d2x = pb.i * spacingMm;
                double d2y = pb.j * spacingMm;
                double rhs = d1x * d2x + d1y * d2y;
                double[] row = new double[] {
                        w1x * w2x,
                        w1x * w2y + w1y * w2x,
                        w1y * w2y
                };
                for (int r = 0; r < 3; r++) {
                    for (int c2 = 0; c2 < 3; c2++) {
                        m[r][c2] += row[r] * row[c2];
                    }
                    b[r] += row[r] * rhs;
                }
                equations++;
            }
        }
        if (equations < 3) {
            throw new Exception("Not enough grid holes were measured to calibrate the machine.");
        }
        double[] g = solve3(m, b);
        double g11 = g[0];
        double g12 = g[1];
        double g22 = g[2];
        if (g11 <= 0.0) {
            throw new Exception("Degenerate grid measurement in X.");
        }
        double l11 = Math.sqrt(g11);
        double l12 = g12 / l11;
        double l22Squared = g22 - l12 * l12;
        if (l22Squared <= 0.0) {
            throw new Exception("Degenerate grid measurement in Y.");
        }
        double l22 = Math.sqrt(l22Squared);
        return new Compensation(l11, l12, l22);
    }

    /**
     * Fit a full 2x2 affine transform A such that A * raw is the ideal (orthogonal, evenly spaced)
     * grid, {@code grid_x = i*spacingMm, grid_y = j*spacingMm}. Unlike {@link #fitCompensation},
     * this also determines the rotation of the grid relative to the raw axes, so all four matrix
     * entries are used. This makes the user coordinate system follow the base plate holes exactly.
     *
     * @return the compensation with all four entries populated
     */
    public static Compensation fitAffine(List<GridMeasurement> measurements, double spacingMm)
            throws Exception {
        GridMeasurement origin = null;
        for (GridMeasurement m : measurements) {
            if (m.i == 0 && m.j == 0) {
                origin = m;
                break;
            }
        }
        if (origin == null) {
            throw new Exception("The reference (0,0) hole was not measured.");
        }
        // Normal equations of the least-squares fit A * W = D, with W the raw displacements and D
        // the ideal grid displacements (both relative to the origin).
        double sxx = 0;
        double sxy = 0;
        double syy = 0;
        double dxwx = 0;
        double dxwy = 0;
        double dywx = 0;
        double dywy = 0;
        int n = 0;
        for (GridMeasurement m : measurements) {
            if (m.i == 0 && m.j == 0) {
                continue;
            }
            double wx = m.rawX - origin.rawX;
            double wy = m.rawY - origin.rawY;
            double dx = m.i * spacingMm;
            double dy = m.j * spacingMm;
            sxx += wx * wx;
            sxy += wx * wy;
            syy += wy * wy;
            dxwx += dx * wx;
            dxwy += dx * wy;
            dywx += dy * wx;
            dywy += dy * wy;
            n++;
        }
        if (n < 3) {
            throw new Exception("Not enough grid holes were measured to calibrate the machine.");
        }
        double det = sxx * syy - sxy * sxy;
        if (Math.abs(det) < 1e-12) {
            throw new Exception("The grid measurements are degenerate. "
                    +"Make sure holes in both X and Y directions were found.");
        }
        // A = (D W^T) * inverse(W W^T).
        double a00 = (dxwx * syy - dxwy * sxy) / det;
        double a01 = (dxwy * sxx - dxwx * sxy) / det;
        double a10 = (dywx * syy - dywy * sxy) / det;
        double a11 = (dywy * sxx - dywx * sxy) / det;
        return new Compensation(a00, a01, a10, a11);
    }

    private static double[] solve3(double[][] a, double[] b) throws Exception {
        double[][] m = new double[3][4];
        for (int i = 0; i < 3; i++) {
            System.arraycopy(a[i], 0, m[i], 0, 3);
            m[i][3] = b[i];
        }
        for (int col = 0; col < 3; col++) {
            int pivot = col;
            for (int r = col + 1; r < 3; r++) {
                if (Math.abs(m[r][col]) > Math.abs(m[pivot][col])) {
                    pivot = r;
                }
            }
            if (Math.abs(m[pivot][col]) < 1e-12) {
                throw new Exception("The grid calibration matrix is singular. "
                        +"Make sure a sufficient number of distinct holes were found.");
            }
            double[] tmp = m[col];
            m[col] = m[pivot];
            m[pivot] = tmp;
            double d = m[col][col];
            for (int c = col; c < 4; c++) {
                m[col][c] /= d;
            }
            for (int r = 0; r < 3; r++) {
                if (r == col) {
                    continue;
                }
                double f = m[r][col];
                if (f == 0.0) {
                    continue;
                }
                for (int c = col; c < 4; c++) {
                    m[r][c] -= f * m[col][c];
                }
            }
        }
        return new double[] {m[0][3], m[1][3], m[2][3]};
    }

    /**
     * Resolve (and create, if needed) the X and Y compensation transform axes with their shared
     * input axes.
     */
    private ReferenceLinearTransformAxis[] getCompensationAxes(ReferenceCamera camera)
            throws Exception {
        // Determine the shared input axes. If a compensation axis already exists, adopt its inputs,
        // otherwise use the axes currently assigned to the camera (which may themselves be
        // transformed axes such as mapped/scaling axes).
        ReferenceLinearTransformAxis existingX = findCompensationTransform(camera, Type.X);
        ReferenceLinearTransformAxis existingY = findCompensationTransform(camera, Type.Y);
        AbstractAxis inputX;
        AbstractAxis inputY;
        if (existingX != null) {
            inputX = existingX.getInputAxisX();
            inputY = existingX.getInputAxisY();
        }
        else if (existingY != null) {
            inputX = existingY.getInputAxisX();
            inputY = existingY.getInputAxisY();
        }
        else {
            inputX = camera.getAxis(Type.X);
            inputY = camera.getAxis(Type.Y);
        }
        ReferenceLinearTransformAxis compX = getOrCreateCompensationAxis(Type.X, inputX, inputY);
        ReferenceLinearTransformAxis compY = getOrCreateCompensationAxis(Type.Y, inputX, inputY);
        return new ReferenceLinearTransformAxis[] {compX, compY};
    }

    /**
     * Apply the full step size (scale) and squareness (shear) correction.
     */
    private void applyCompensation(ReferenceCamera camera, Compensation compensation) throws Exception {
        ReferenceLinearTransformAxis[] axes = getCompensationAxes(camera);
        axes[0].setFactorX(compensation.l11);
        axes[0].setFactorY(compensation.l12);
        axes[0].setOffset(new Length(0.0, LengthUnit.Millimeters));
        axes[0].setCompensation(true);
        axes[1].setFactorX(compensation.l21);
        axes[1].setFactorY(compensation.l22);
        axes[1].setOffset(new Length(0.0, LengthUnit.Millimeters));
        axes[1].setCompensation(true);
    }

    private ReferenceLinearTransformAxis getOrCreateCompensationAxis(Type type, AbstractAxis inputX,
            AbstractAxis inputY) throws Exception {
        // Reuse an existing compensation axis with the same input axes, if present.
        for (Axis axis : machine.getAxes()) {
            if (axis instanceof ReferenceLinearTransformAxis && axis.getType() == type) {
                ReferenceLinearTransformAxis transform = (ReferenceLinearTransformAxis) axis;
                if (transform.isCompensation()
                        && transform.getInputAxisX() == inputX
                        && transform.getInputAxisY() == inputY) {
                    return transform;
                }
            }
        }
        // None exists: create it and insert it into the axis chain.
        AbstractAxis primary = (type == Type.X ? inputX : inputY);
        ReferenceLinearTransformAxis transform = new ReferenceLinearTransformAxis();
        transform.setType(type);
        transform.setName(primary.getName()+"-square");
        transform.setInputAxisX(inputX);
        transform.setInputAxisY(inputY);
        transform.setFactorX(type == Type.X ? 1.0 : 0.0);
        transform.setFactorY(type == Type.Y ? 1.0 : 0.0);
        transform.setCompensation(true);
        machine.addAxis(transform);
        // Make the input axis name distinguishable.
        if (!primary.getName().endsWith("-raw")) {
            primary.setName(primary.getName()+"-raw");
        }
        // Reassign all HeadMountables that currently use the input axis directly.
        for (Head head : machine.getHeads()) {
            for (HeadMountable hm : head.getHeadMountables()) {
                if (hm.getAxis(type) == primary) {
                    ((AbstractHeadMountable) hm).setAxis(transform, type);
                }
            }
        }
        return transform;
    }

    /**
     * @return the compensation currently applied to the camera's X/Y axis chain, or null if none.
     */
    private Compensation getCurrentCompensation(ReferenceCamera camera) {
        ReferenceLinearTransformAxis compX = findCompensationTransform(camera, Type.X);
        ReferenceLinearTransformAxis compY = findCompensationTransform(camera, Type.Y);
        if (compX == null && compY == null) {
            return null;
        }
        double l11 = compX != null ? compX.getFactorX() : 1.0;
        double l12 = compX != null ? compX.getFactorY() : 0.0;
        double l21 = compY != null ? compY.getFactorX() : 0.0;
        double l22 = compY != null ? compY.getFactorY() : 1.0;
        return new Compensation(l11, l12, l21, l22);
    }

    private void restoreCompensation(ReferenceCamera camera, Compensation compensation) {
        ReferenceLinearTransformAxis compX = findCompensationTransform(camera, Type.X);
        ReferenceLinearTransformAxis compY = findCompensationTransform(camera, Type.Y);
        double l11 = compensation != null ? compensation.l11 : 1.0;
        double l12 = compensation != null ? compensation.l12 : 0.0;
        double l21 = compensation != null ? compensation.l21 : 0.0;
        double l22 = compensation != null ? compensation.l22 : 1.0;
        if (compX != null) {
            compX.setFactorX(l11);
            compX.setFactorY(l12);
        }
        if (compY != null) {
            compY.setFactorX(l21);
            compY.setFactorY(l22);
        }
    }

    private ReferenceLinearTransformAxis findCompensationTransform(ReferenceCamera camera, Type type) {
        AbstractAxis axis = camera.getAxis(type);
        while (axis instanceof ReferenceLinearTransformAxis) {
            ReferenceLinearTransformAxis transform = (ReferenceLinearTransformAxis) axis;
            if (transform.isCompensation()) {
                return transform;
            }
            axis = transform.getPrimaryInputAxis();
        }
        return null;
    }

    private static String format(double value) {
        if (value == Math.rint(value)) {
            return String.format("%.0f", value);
        }
        return String.format("%.3f", value);
    }
}
