package org.firstinspires.ftc.teamcode.util;

/**
 * ShooterBallistics
 * <p>
 * PURPOSE
 * -------
 * Converts a measured distance to the target and the current turret angle
 * into a shooter setpoint (hood position + flywheel RPM).
 * <p>
 * This class is entirely LUT-based (Lookup Tables), derived from empirical
 * shooting data collected on the real robot.
 * <p>
 * <p>
 * PIPELINE OVERVIEW
 * -----------------
 * <p>
 * 1) INPUTS
 * - distanceInches : Horizontal ground distance from robot to target
 * - turretAngleDeg : Current turret rotation relative to robot (degrees)
 * <p>
 * 2) HOOD SELECTION (DISCRETE MODES)
 * - Hood position is treated as a discrete mode, NOT a continuous variable.
 * - Each hood position has its own ballistics behavior.
 * - Distance thresholds select which hood is appropriate.
 * <p>
 * 3) DISTANCE → RPM LOOKUP (LUT + INTERPOLATION)
 * - For the selected hood, a distance→RPM lookup table is used.
 * - Linear interpolation fills gaps between measured data points.
 * - This avoids fragile physics formulas and matches real-world behavior.
 * <p>
 * 4) TURRET ANGLE COMPENSATION (ENERGY LOSS)
 * - Turret rotation does NOT change geometry via cosine projection.
 * - Instead, it introduces mechanical and feeding losses.
 * - These losses are handled via quadrant-based RPM scaling factors.
 * <p>
 * 5) OUTPUT
 * - A ShooterSetpoint containing:
 * • Flywheel RPM
 * • Hood servo position
 * <p>
 * <p>
 * DESIGN GOALS
 * ------------
 * - Deterministic (no runtime fitting)
 * - Fast (no trig, no allocations)
 * - Tunable between matches
 * - Empirically accurate
 */
public final class ShooterBallistics {

    /**
     * Immutable shooter setpoint container.
     */
    public static class ShooterSetpoint {
        public final double rpm;
        public final double hood;

        public ShooterSetpoint(double rpm, double hood) {
            this.rpm = rpm;
            this.hood = hood;
        }
    }

    // =========================
    // Hood positions (servo)
    // =========================
    public static final double HOOD_CLOSE = 0.50;
    public static final double HOOD_MID = 0.44;
    public static final double HOOD_FAR = 0.38;

    // =========================
    // LUTs: Distance (in) → RPM
    // These are empirical averages from real shots
    // =========================

    // Hood = 0.50
    private static final double[] DIST_050 = {
            80, 90, 100, 120, 130, 140, 150, 160, 170, 180
    };
    private static final double[] RPM_050 = {
            2859, 2923, 2993, 3812, 3806, 3874, 4072, 4190, 4349, 4350
    };

    // Hood = 0.44
    private static final double[] DIST_044 = {
            60, 70, 80, 90, 100, 130, 140, 150, 160, 170, 180, 190, 200, 210
    };
    private static final double[] RPM_044 = {
            2360, 2281, 2854, 2899, 3020, 3910, 3772, 3965, 4069, 4304, 4319, 4612, 4571, 4638
    };

    // Hood = 0.38
    private static final double[] DIST_038 = {
            70, 80, 90, 100, 110, 160, 170, 180, 190, 200
    };
    private static final double[] RPM_038 = {
            2492, 2547, 2975, 3066, 3077, 4099, 4059, 4557, 4603, 4889
    };

    // =========================
    // Public API
    // =========================

    /**
     * Main entry point used by OpModes.
     *
     * @param distanceInches Distance from robot to target (ground distance)
     * @param turretAngleDeg Turret angle relative to robot (degrees)
     * @return ShooterSetpoint containing RPM and hood position
     */
    public static ShooterSetpoint getSetpoint(
            double distanceInches,
            double turretAngleDeg
    ) {
        // 1) Select hood mode based on distance
        double hood = selectHood(distanceInches);

        // 2) Lookup required RPM for that hood and distance
        double rpm = lookupRPM(distanceInches, hood);

        // 3) Apply turret-angle compensation (energy loss, not geometry)
        rpm *= turretCompensation(turretAngleDeg, hood);

        return new ShooterSetpoint(rpm, hood);
    }

    // =========================
    // Hood selection logic
    // =========================
    private static double selectHood(double distance) {
        if (distance < 120) {
            return HOOD_CLOSE;
        } else if (distance < 165) {
            return HOOD_MID;
        } else {
            return HOOD_FAR;
        }
    }

    // =========================
    // LUT lookup: distance → RPM
    // =========================
    private static double lookupRPM(double distance, double hood) {
        if (hood == HOOD_CLOSE) {
            return interpolate(distance, DIST_050, RPM_050);
        }
        if (hood == HOOD_MID) {
            return interpolate(distance, DIST_044, RPM_044);
        }
        if (hood == HOOD_FAR) {
            return interpolate(distance, DIST_038, RPM_038);
        }
        return 0.0;
    }

    // =========================
    // Turret compensation
    // Hood + quadrant based
    // Empirically derived
    // =========================
    private static double turretCompensation(double angleDeg, double hood) {
        double a = Math.abs(angleDeg);

        boolean q0  = a <= 22.5;
        boolean q45 = a > 22.5 && a <= 67.5;
        boolean q90 = a > 67.5;

        // ---- Hood = 0.50 (CLOSE) ----
        if (hood == HOOD_CLOSE) {
            if (q0)  return 1.0000;
            if (q45) return 1.0474;
            return 1.0755;
        }

        // ---- Hood = 0.44 (MID) ----
        if (hood == HOOD_MID) {
            if (q0)  return 1.0000;
            if (q45) return 1.0522;
            return 1.0651;
        }

        // ---- Hood = 0.38 (FAR) ----
        if (hood == HOOD_FAR) {
            if (q0)  return 1.0000;
            if (q45) return 0.9553;
            return 0.9817;
        }

        return 1.0;
    }

    // =========================
    // Linear interpolation
    // =========================
    private static double interpolate(
            double x,
            double[] xVals,
            double[] yVals
    ) {
        if (x <= xVals[0]) return yVals[0];
        if (x >= xVals[xVals.length - 1]) return yVals[yVals.length - 1];

        for (int i = 0; i < xVals.length - 1; i++) {
            if (x >= xVals[i] && x <= xVals[i + 1]) {
                double t = (x - xVals[i]) / (xVals[i + 1] - xVals[i]);
                return yVals[i] + t * (yVals[i + 1] - yVals[i]);
            }
        }
        return yVals[yVals.length - 1];
    }
}
