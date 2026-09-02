package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

import java.util.ArrayList;
import java.util.List;

/**
 * A utility class to manage a HuskyLens camera specifically for detecting colored balls.
 * It identifies the "best" ball (largest and most centered) and identifies clusters
 * of balls to locate high-density collection zones.
 */
public class HuskyLensUtil {

    /**
     * A simple data class to hold information about a detected ball.
     */
    public static class BallData {
        public final int id;         // Color ID (e.g., 1 for Green, 2 for Purple)
        public final int x;          // Horizontal center of the ball on the screen (0-319)
        public final int y;          // Vertical center of the ball on the screen (0-239)
        public final int width;      // Width of the detected blob in pixels
        public final int height;     // Height of the detected blob in pixels

        public BallData(HuskyLens.Block block) {
            this.id = block.id;
            this.x = block.x;
            this.y = block.y;
            this.width = block.width;
            this.height = block.height;
        }

        /**
         * Provides a simple string representation for telemetry.
         */
        @Override
        public String toString() {
            return String.format("ID: %d, Pos: (%d, %d), Size: %dx%d", id, x, y, width, height);
        }
    }

    /**
     * A data class representing a group of balls located close together.
     */
    public static class ClusterData {
        public final List<BallData> balls = new ArrayList<>();
        public int centerX;
        public int centerY;
        public int totalArea;

        public void addBall(BallData ball) {
            balls.add(ball);
            calculateMetrics();
        }

        private void calculateMetrics() {
            int sumX = 0, sumY = 0, sumArea = 0;
            for (BallData b : balls) {
                sumX += b.x;
                sumY += b.y;
                sumArea += (b.width * b.height);
            }
            centerX = sumX / balls.size();
            centerY = sumY / balls.size();
            totalArea = sumArea;
        }

        public int getBallCount() {
            return balls.size();
        }
    }

    private final HuskyLens huskyLens;

    // The "best" ball is the one we are most confident is centered and ready for intake.
    private BallData bestBall = null;

    // The "best cluster" represents the highest density of balls for collection.
    private ClusterData bestCluster = null;

    // Center of the HuskyLens screen resolution (320x240)
    private static final int SCREEN_CENTER_X = 160;

    // Maximum distance in pixels between balls to be considered part of a cluster
    private static final int CLUSTER_PROXIMITY_THRESHOLD = 85;

    /**
     * Initializes the HuskyLens camera.
     * @param robot The RobotHardware instance to get the camera from.
     */
    public HuskyLensUtil(RobotHardware robot) {
        this.huskyLens = robot.huskyLens;

        // Ensure the camera is communicating
        if (huskyLens.knock()) {
            // Set the algorithm to COLOR_RECOGNITION
            huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        }
    }

    /**
     * Processes camera data to identify the best individual target and the best cluster.
     * Must be called in every loop of the OpMode.
     */
    public void update() {
        HuskyLens.Block[] blocks = huskyLens.blocks();

        // Reset state
        bestBall = null;
        bestCluster = null;

        if (blocks.length == 0) return;

        List<BallData> allBalls = new ArrayList<>();
        for (HuskyLens.Block block : blocks) {
            allBalls.add(new BallData(block));
        }

        // --- Find Best Individual Ball ---
        int bestBallScore = -1;
        for (BallData ball : allBalls) {
            int area = ball.width * ball.height;
            int distFromCenter = Math.abs(ball.x - SCREEN_CENTER_X);
            int score = area - (distFromCenter * 2);

            if (score > bestBallScore) {
                bestBallScore = score;
                bestBall = ball;
            }
        }

        // --- Identify Best Cluster ---
        List<ClusterData> clusters = new ArrayList<>();
        boolean[] assigned = new boolean[allBalls.size()];

        for (int i = 0; i < allBalls.size(); i++) {
            if (assigned[i]) continue;
            ClusterData cluster = new ClusterData();
            groupNeighbors(i, allBalls, assigned, cluster);
            clusters.add(cluster);
        }

        double maxClusterScore = -1000;
        for (ClusterData c : clusters) {
            int distToCenter = Math.abs(c.centerX - SCREEN_CENTER_X);
            // Cluster score gives heavy weight to ball count to prioritize groups over singletons
            double ballCountWeight = (c.getBallCount() - 1) * 4000;
            double score = c.totalArea + ballCountWeight - (distToCenter * 2);

            if (score > maxClusterScore) {
                maxClusterScore = score;
                bestCluster = c;
            }
        }
    }

    /**
     * Recursive neighbor finding to build clusters based on proximity.
     */
    private void groupNeighbors(int index, List<BallData> allBalls, boolean[] assigned, ClusterData cluster) {
        assigned[index] = true;
        BallData current = allBalls.get(index);
        cluster.addBall(current);

        for (int i = 0; i < allBalls.size(); i++) {
            if (!assigned[i]) {
                BallData other = allBalls.get(i);
                double dist = Math.hypot(current.x - other.x, current.y - other.y);
                if (dist < CLUSTER_PROXIMITY_THRESHOLD) {
                    groupNeighbors(i, allBalls, assigned, cluster);
                }
            }
        }
    }

    /**
     * Checks if any balls are detected.
     */
    public boolean isBallDetected() {
        return bestBall != null;
    }

    /**
     * Checks if a cluster (more than one ball) is detected.
     */
    public boolean isClusterDetected() {
        return bestCluster != null && bestCluster.getBallCount() > 1;
    }

    /**
     * Gets the best individual ball data.
     */
    public BallData getBestBall() {
        return bestBall;
    }

    /**
     * Gets the data for the best cluster found.
     */
    public ClusterData getBestCluster() {
        return bestCluster;
    }

    /**
     * Adds comprehensive telemetry for both single targets and clusters.
     */
    public void addTelemetry(Telemetry telemetry) {
        if (isClusterDetected()) {
            telemetry.addData("Intake Vision", "CLUSTER FOUND (%d balls)", bestCluster.getBallCount());
            telemetry.addData("  > Cluster Center", "%d, %d", bestCluster.centerX, bestCluster.centerY);
            telemetry.addData("  > Total Area", bestCluster.totalArea);
        } else if (isBallDetected()) {
            telemetry.addData("Intake Vision", "Single Target Locked");
            telemetry.addData("  > Ball Data", bestBall.toString());
        } else {
            telemetry.addData("Intake Vision", "No Objects Detected");
        }
    }
}