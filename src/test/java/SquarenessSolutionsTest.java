import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.openpnp.machine.reference.solutions.SquarenessSolutions;
import org.openpnp.machine.reference.solutions.SquarenessSolutions.Compensation;
import org.openpnp.machine.reference.solutions.SquarenessSolutions.GridMeasurement;

/**
 * Tests the pure math of the base plate hole grid squareness/step calibration.
 */
public class SquarenessSolutionsTest {

    private static final double SPACING = 32.0;

    /**
     * Build the raw axis coordinates of a regular hole grid as an imperfect machine would measure
     * them. The physical position of hole (i, j) is assumed to be (i*spacing, j*spacing). The raw
     * coordinates are obtained by applying the inverse of the physical mapping matrix p (2x2,
     * column-major {m00, m01, m10, m11}) to the physical position.
     */
    private static List<GridMeasurement> buildMeasurements(double[] p,
            int minI, int maxI, int minJ, int maxJ) {
        List<int[]> indices = new ArrayList<>();
        for (int j = minJ; j <= maxJ; j++) {
            for (int i = minI; i <= maxI; i++) {
                // Only the cross/axis holes are typically measured, keep the same shape.
                if (i != 0 && j != 0) {
                    continue;
                }
                indices.add(new int[] {i, j});
            }
        }
        return buildMeasurements(p, indices);
    }

    /**
     * Build measurements for an explicit set of grid indices, e.g. to simulate occupied holes that
     * are missing from the measurement.
     */
    private static List<GridMeasurement> buildMeasurements(double[] p, List<int[]> indices) {
        double det = p[0] * p[3] - p[1] * p[2];
        double[] inv = new double[] {
                p[3] / det, -p[1] / det,
                -p[2] / det, p[0] / det
        };
        List<GridMeasurement> measurements = new ArrayList<>();
        for (int[] index : indices) {
            int i = index[0];
            int j = index[1];
            double gx = i * SPACING;
            double gy = j * SPACING;
            double rawX = inv[0] * gx + inv[1] * gy;
            double rawY = inv[2] * gx + inv[3] * gy;
            measurements.add(new GridMeasurement(i, j, rawX, rawY));
        }
        return measurements;
    }

    private static double[] correct(Compensation c, GridMeasurement m) {
        return new double[] {
                c.l11 * m.rawX + c.l12 * m.rawY,
                c.l22 * m.rawY
        };
    }

    @Test
    public void testScaleAndShearAreCorrected() throws Exception {
        // 1% X scale error, 2% Y scale error and a shear of 0.05.
        double[] p = new double[] {
                1.01, 0.05,
                0.00, 1.02
        };
        List<GridMeasurement> measurements = buildMeasurements(p, 0, 4, 0, 4);
        Compensation c = SquarenessSolutions.fitCompensation(measurements, SPACING);

        // Since the physical mapping is upper triangular, the correction should invert it exactly,
        // i.e. the corrected grid must be the ideal orthogonal grid.
        for (GridMeasurement m : measurements) {
            double[] corrected = correct(c, m);
            assertEquals(m.i * SPACING, corrected[0], 1e-9,
                    "X of hole (" + m.i + "," + m.j + ")");
            assertEquals(m.j * SPACING, corrected[1], 1e-9,
                    "Y of hole (" + m.i + "," + m.j + ")");
        }
    }

    @Test
    public void testSkippedHolesStillCalibrate() throws Exception {
        // Same imperfect machine as above, but holes (1,0), (3,0) and (0,2) are occupied and were
        // skipped, so only the following holes were measured.
        double[] p = new double[] {
                1.01, 0.05,
                0.00, 1.02
        };
        List<int[]> indices = new ArrayList<>();
        indices.add(new int[] {0, 0});
        indices.add(new int[] {2, 0});
        indices.add(new int[] {4, 0});
        indices.add(new int[] {0, 1});
        indices.add(new int[] {0, 3});
        indices.add(new int[] {0, 4});
        List<GridMeasurement> measurements = buildMeasurements(p, indices);
        Compensation c = SquarenessSolutions.fitCompensation(measurements, SPACING);

        // The correction must still invert the upper triangular physical mapping exactly.
        double maxError = 0;
        for (GridMeasurement m : measurements) {
            double[] corrected = correct(c, m);
            maxError = Math.max(maxError, Math.abs(corrected[0] - m.i * SPACING));
            maxError = Math.max(maxError, Math.abs(corrected[1] - m.j * SPACING));
        }
        assertEquals(0.0, maxError, 1e-9,
                "Skipped holes must not affect the calibration result");
    }

    @Test
    public void testRotatedGridIsOrthogonalizedWithoutRotationCompensation() throws Exception {
        // The plate is rotated by 5 degrees in addition to scale and shear. The correction must
        // still yield an orthogonal/evenly spaced grid, but it must not undo the plate rotation.
        double angle = Math.toRadians(5.0);
        double[] rotation = new double[] {
                Math.cos(angle), -Math.sin(angle),
                Math.sin(angle), Math.cos(angle)
        };
        double[] scaleShear = new double[] {
                1.02, 0.04,
                0.00, 0.98
        };
        double[] p = multiply(rotation, scaleShear);

        List<GridMeasurement> measurements = buildMeasurements(p, 0, 4, 0, 4);
        Compensation c = SquarenessSolutions.fitCompensation(measurements, SPACING);

        double[] origin = correct(c, find(measurements, 0, 0));
        double[] xHole = correct(c, find(measurements, 4, 0));
        double[] yHole = correct(c, find(measurements, 0, 4));

        double ux = xHole[0] - origin[0];
        double uy = xHole[1] - origin[1];
        double vx = yHole[0] - origin[0];
        double vy = yHole[1] - origin[1];

        // Orthogonality.
        assertEquals(0.0, ux * vx + uy * vy, 1e-6, "Grid must be orthogonal after correction");
        // Even spacing, the correction only heals scale and shear - the grid is rotated by 5 deg.
        assertEquals(4 * SPACING, Math.hypot(ux, uy), 1e-6, "X pitch");
        assertEquals(4 * SPACING, Math.hypot(vx, vy), 1e-6, "Y pitch");
    }

    @Test
    public void testAffineFitInvertsFullMapping() throws Exception {
        // Rotation + scale + shear. The full affine fit must invert the mapping exactly, so the
        // corrected grid becomes axis aligned (the rotation is undone as well).
        double angle = Math.toRadians(3.0);
        double[] rotation = new double[] {
                Math.cos(angle), -Math.sin(angle),
                Math.sin(angle), Math.cos(angle)
        };
        double[] scaleShear = new double[] {
                1.02, 0.03,
                0.01, 0.98
        };
        double[] p = multiply(rotation, scaleShear);

        List<GridMeasurement> measurements = buildMeasurements(p, -4, 4, 0, 4);
        Compensation c = SquarenessSolutions.fitAffine(measurements, SPACING);

        for (GridMeasurement m : measurements) {
            double x = c.l11 * m.rawX + c.l12 * m.rawY;
            double y = c.l21 * m.rawX + c.l22 * m.rawY;
            assertEquals(m.i * SPACING, x, 1e-9, "X of hole (" + m.i + "," + m.j + ")");
            assertEquals(m.j * SPACING, y, 1e-9, "Y of hole (" + m.i + "," + m.j + ")");
        }
    }

    @Test
    public void testCalibrationDataPersistence() throws Exception {
        List<GridMeasurement> measurements = buildMeasurements(
                new double[] {1.01, 0.02, 0.0, 1.02}, 0, 3, 0, 3);
        SquarenessSolutions.GridCalibrationData data =
                new SquarenessSolutions.GridCalibrationData(32.0, "H1", 40.0, measurements);

        org.simpleframework.xml.Serializer serializer =
                org.openpnp.model.Configuration.createSerializer();
        java.io.StringWriter writer = new java.io.StringWriter();
        serializer.write(data, writer);
        SquarenessSolutions.GridCalibrationData read =
                serializer.read(SquarenessSolutions.GridCalibrationData.class, writer.toString());

        assertEquals(32.0, read.getSpacingMm(), 1e-9);
        assertEquals("H1", read.getHeadName());
        assertEquals(40.0, read.getPlateZMm(), 1e-9);
        List<GridMeasurement> readMeasurements = read.toMeasurements();
        assertEquals(measurements.size(), readMeasurements.size());
        for (int k = 0; k < measurements.size(); k++) {
            assertEquals(measurements.get(k).i, readMeasurements.get(k).i);
            assertEquals(measurements.get(k).j, readMeasurements.get(k).j);
            assertEquals(measurements.get(k).rawX, readMeasurements.get(k).rawX, 1e-9);
            assertEquals(measurements.get(k).rawY, readMeasurements.get(k).rawY, 1e-9);
        }
    }

    @Test
    public void testColumnsRowsPersistence() throws Exception {
        SquarenessSolutions solutions = new SquarenessSolutions();
        solutions.setHoleColumns(12);
        solutions.setHoleRows(6);

        org.simpleframework.xml.Serializer serializer =
                org.openpnp.model.Configuration.createSerializer();
        java.io.StringWriter writer = new java.io.StringWriter();
        serializer.write(solutions, writer);
        String xml = writer.toString();
        org.junit.jupiter.api.Assertions.assertTrue(xml.contains("hole-columns=\"12\""), xml);
        org.junit.jupiter.api.Assertions.assertTrue(xml.contains("hole-rows=\"6\""), xml);

        SquarenessSolutions read = serializer.read(SquarenessSolutions.class, xml);
        assertEquals(12, read.getHoleColumns());
        assertEquals(6, read.getHoleRows());
    }

    @Test
    public void testResidualGridImprovesLocalAccuracy() throws Exception {
        // A machine with a position dependent X scale (a left-to-right gradient), like a belt driven
        // machine, plus a small Y scale error. The affine fit can only represent the average; the
        // residual grid must recover the local variation.
        List<GridMeasurement> measurements = new ArrayList<>();
        for (int j = 0; j <= 4; j++) {
            for (int i = 0; i <= 9; i++) {
                double idealX = i * SPACING;
                double idealY = j * SPACING;
                // 0.5% scale gradient across X, 0.2% Y scale error.
                double rawX = idealX * (1.0 - 0.005 * (i - 4.5) / 4.5);
                double rawY = idealY * 1.002;
                measurements.add(new GridMeasurement(i, j, rawX, rawY));
            }
        }
        Compensation affine = SquarenessSolutions.fitAffine(measurements, SPACING);
        org.openpnp.machine.reference.axis.ReferenceLinearTransformAxis.GridResidual gridX =
                SquarenessSolutions.buildGridResidual(measurements, affine, true, SPACING);

        double maxLinearError = 0;
        double maxCorrectedError = 0;
        for (GridMeasurement m : measurements) {
            double linear = affine.l11 * m.rawX + affine.l12 * m.rawY;
            double corrected = linear + gridX.evaluate(m.rawX, m.rawY);
            double target = m.i * SPACING;
            maxLinearError = Math.max(maxLinearError, Math.abs(linear - target));
            maxCorrectedError = Math.max(maxCorrectedError, Math.abs(corrected - target));
        }
        org.junit.jupiter.api.Assertions.assertTrue(maxCorrectedError < maxLinearError,
                "residual grid must improve on the affine (linear " + maxLinearError
                + ", corrected " + maxCorrectedError + ")");
        org.junit.jupiter.api.Assertions.assertTrue(maxCorrectedError < 0.02,
                "residual grid must be accurate at the holes (" + maxCorrectedError + ")");
    }

    @Test
    public void testGridResidualPersistence() throws Exception {
        org.openpnp.machine.reference.axis.ReferenceLinearTransformAxis.GridResidual grid =
                new org.openpnp.machine.reference.axis.ReferenceLinearTransformAxis.GridResidual(
                        32.0, 1.01, 0.0, 0.0, 1.0, 0.0, 0.0);
        grid.addNode(0, 0, 0.001);
        grid.addNode(1, 0, -0.002);
        grid.addNode(2, 0, 0.003);

        org.simpleframework.xml.Serializer serializer =
                org.openpnp.model.Configuration.createSerializer();
        java.io.StringWriter writer = new java.io.StringWriter();
        serializer.write(grid, writer);
        org.openpnp.machine.reference.axis.ReferenceLinearTransformAxis.GridResidual read =
                serializer.read(
                        org.openpnp.machine.reference.axis.ReferenceLinearTransformAxis.GridResidual.class,
                        writer.toString());
        assertEquals(3, read.nodeCount());
        // The residual at a node must survive the round trip (raw 64/1.01 maps to ideal X = 64).
        assertEquals(0.003, read.evaluate(64.0 / 1.01, 0.0), 1e-4);
    }

    @Test
    public void testIdealMachineIsUnchanged() throws Exception {
        List<GridMeasurement> measurements = buildMeasurements(
                new double[] {1, 0, 0, 1}, 0, 4, 0, 4);
        Compensation c = SquarenessSolutions.fitCompensation(measurements, SPACING);
        assertEquals(1.0, c.l11, 1e-9);
        assertEquals(0.0, c.l12, 1e-9);
        assertEquals(1.0, c.l22, 1e-9);
    }

    private static GridMeasurement find(List<GridMeasurement> measurements, int i, int j) {
        for (GridMeasurement m : measurements) {
            if (m.i == i && m.j == j) {
                return m;
            }
        }
        throw new IllegalArgumentException("No measurement (" + i + "," + j + ")");
    }

    private static double[] multiply(double[] a, double[] b) {
        return new double[] {
                a[0] * b[0] + a[1] * b[2],
                a[0] * b[1] + a[1] * b[3],
                a[2] * b[0] + a[3] * b[2],
                a[2] * b[1] + a[3] * b[3]
        };
    }
}
