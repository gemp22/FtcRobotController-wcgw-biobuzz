package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.UdpClientPlot;

import java.util.LinkedList;
import java.util.Locale;

public class BallColor {
    // --- DEBUGGING ---
    private boolean isDebugEnabled = false;
    private UdpClientPlot plotClient;
    private String plotClientHost = "192.168.43.100";
    private int plotClientPort = 7778; // Note: Same port as FlyWheel, might need to change if run simultaneously on separate plotters

    /**
     * Enum to represent the detected color of the ball.
     */
    public enum DetectedColor {
        NA,
        NONE,
        GREEN,
        PURPLE
    }

    // Constants
    private static final int ROLLING_WINDOW_SIZE = 10;
    private static final double MAX_DETECTION_DISTANCE_CM = 7.0;

    // Hardware
    private final NormalizedColorSensor colorSensor;
    private final DistanceSensor distanceSensor;

    // Data storage
    private final LinkedList<double[]> sensorReadings;

    /**
     * Constructor for the BallColor class.
     */
    public BallColor(NormalizedColorSensor sensor) {
        this.colorSensor = sensor;
        if (sensor instanceof DistanceSensor) {
            this.distanceSensor = (DistanceSensor) sensor;
        } else {
            this.distanceSensor = null;
        }
        this.sensorReadings = new LinkedList<>();
    }

    /**
     * Enables or disables the UDP plot client for debugging.
     * @param enable True to enable debugging, false to disable.
     */
    public void setDebug(boolean enable) {
        if (enable && !this.isDebugEnabled) {
            // Turning debugging ON
            this.isDebugEnabled = true;
            if (plotClient == null) {
                plotClient = new UdpClientPlot(plotClientHost, plotClientPort);
                if (plotClient.isInitialized()) {
                    long initTime = System.currentTimeMillis();
                    // Setup Left Y-Axis (Y1) for Color
                    plotClient.sendYLimits(initTime, 1.0, 0.0);
                    plotClient.sendYUnits(initTime + 1, "Color");

                    // *** Setup Right Y-Axis (Y2) for Distance ***
                    plotClient.sendYLimits2(initTime + 2, 15.0, 0.0); // Set Y2 axis from 0 to 15 cm
                    plotClient.sendYUnits2(initTime + 3, "Distance (cm)");    // Set Y2 axis label

                    // Left Y-Axis Series
                    plotClient.sendSeriesNameLine(initTime + 4, "Avg. Red", 1);
                    plotClient.sendSeriesNameLine(initTime + 5, "Avg. Blue", 2);
                    plotClient.sendSeriesNameLine(initTime + 6, "Avg. Green", 3);

                    // Right Y-Axis Series
                    plotClient.sendSeriesNameLine2(initTime + 7, "Distance", 5);

                    // *** Add a marker to show when debug was enabled ***
                    plotClient.sendTextMarker(initTime + 4, "BallColor Debug Enabled", "top");
                }
            }
        } else if (!enable && this.isDebugEnabled) {
            // Turning debugging OFF
            this.isDebugEnabled = false;
            close(); // Close the client when disabling
        }
    }

    /**
     * Updates the sensor readings. Should be called in a loop.
     */
    public void update() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        double distance = (distanceSensor != null) ? distanceSensor.getDistance(DistanceUnit.CM) : -1.0;

        sensorReadings.add(new double[]{colors.red, colors.green, colors.blue, distance});

        if (sensorReadings.size() > ROLLING_WINDOW_SIZE) {
            sensorReadings.removeFirst();
        }
    }

    /**
     * Analyzes the rolling window to determine the ball color.
     * @return DetectedColor enum (NONE, GREEN, or PURPLE).
     */
    public DetectedColor getBallColor() {
        if (sensorReadings.size() < ROLLING_WINDOW_SIZE) {
            if (isDebugEnabled && plotClient != null) {
                long time = System.currentTimeMillis();
                plotClient.sendLineY(time, 0, 1); // avgRed
                plotClient.sendLineY(time, 0, 2); // avgBlue
                plotClient.sendLineY(time, 0, 3); // avgGreen
                plotClient.sendLineY2(time, 0, 5); // avgDistance
                plotClient.sendKeyValue(time, "detected_color", "NONE (waiting for data)");
            }
            return DetectedColor.NONE;
        }

        double sumRed = 0, sumGreen = 0, sumBlue = 0, sumDistance = 0;
        int validDistanceReadings = 0;
        for (double[] reading : sensorReadings) {
            sumRed += reading[0];
            sumGreen += reading[1];
            sumBlue += reading[2];
            if (reading[3] >= 0) {
                sumDistance += reading[3];
                validDistanceReadings++;
            }
        }

        double avgRed = sumRed / ROLLING_WINDOW_SIZE;
        double avgGreen = sumGreen / ROLLING_WINDOW_SIZE;
        double avgBlue = sumBlue / ROLLING_WINDOW_SIZE;
        double avgDistance = (validDistanceReadings > 0) ? (sumDistance / validDistanceReadings) : Double.MAX_VALUE;

        DetectedColor result;

        // --- Detection Logic ---
        if (distanceSensor == null || avgDistance >= MAX_DETECTION_DISTANCE_CM) {
            result = DetectedColor.NONE;
        } else if (avgBlue > avgGreen && avgBlue > avgRed) {
            result = DetectedColor.PURPLE;
        } else if (avgGreen > avgRed * 1.2 && avgGreen > avgBlue) {
            result = DetectedColor.GREEN;
        } else {
            result = DetectedColor.NONE;
        }

        // --- DEBUG Plotting ---
        if (isDebugEnabled && plotClient != null) {
            long time = System.currentTimeMillis();

            // Plot colors on the left Y-Axis
            plotClient.sendLineY(time, avgRed, 1);
            plotClient.sendLineY(time, avgBlue, 2);
            plotClient.sendLineY(time, avgGreen, 3);

            // *** NEW: Plot raw distance on the right Y-Axis (Y2) ***
            double plotDistance = (avgDistance == Double.MAX_VALUE) ? 15.0 : avgDistance; // Use 15 if out of range
            plotClient.sendLineY2(time, plotDistance, 5); // Style 5 for Distance

            // Send data to key-value table
            plotClient.sendKeyValue(time, "detected_color", result.toString());
            plotClient.sendKeyValue(time, "detected_distance", String.format(Locale.US, "%.2f cm", avgDistance));
        }

        return result;
    }

    /**
     * Allows setting the gain of the color sensor.
     */
    public void setGain(float gain) {
        colorSensor.setGain(gain);
    }

    /**
     * Closes any open resources, like the UDP client.
     */
    public void close() {
        if (plotClient != null) {
            plotClient.close();
            plotClient = null;
        }
        this.isDebugEnabled = false;
    }
}



//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
//import com.qualcomm.robotcore.hardware.NormalizedRGBA;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import java.util.LinkedList;
//
//public class BallColor {
//    // --- DEBUGGING ---
//    // MODIFIED: Replaced static final boolean with an instance variable
//    private boolean isDebugEnabled = false;
//    private UdpClientPlot plotClient;
//    private String plotClientHost = "192.168.43.100";
//    private int plotClientPort = 7778; // Note: Same port as FlyWheel, might need to change if run simultaneously on separate plotters
//
//    /**
//     * Enum to represent the detected color of the ball.
//     */
//    public enum DetectedColor {
//        NONE,
//        GREEN,
//        PURPLE
//    }
//
//    // Constants
//    private static final int ROLLING_WINDOW_SIZE = 10;
//    private static final double MAX_DETECTION_DISTANCE_CM = 7.0;
//
//    // Hardware
//    private final NormalizedColorSensor colorSensor;
//    private final DistanceSensor distanceSensor;
//
//    // Data storage
//    private final LinkedList<double[]> sensorReadings;
//
//    /**
//     * Constructor for the BallColor class.
//     */
//    public BallColor(NormalizedColorSensor sensor) {
//        this.colorSensor = sensor;
//        if (sensor instanceof DistanceSensor) {
//            this.distanceSensor = (DistanceSensor) sensor;
//        } else {
//            this.distanceSensor = null;
//        }
//        this.sensorReadings = new LinkedList<>();
//
//        // MODIFIED: Debug client is no longer initialized here by default.
//    }
//
//    /**
//     * NEW: Enables or disables the UDP plot client for debugging.
//     * @param enable True to enable debugging, false to disable.
//     */
//    public void setDebug(boolean enable) {
//        if (enable && !this.isDebugEnabled) {
//            // Turning debugging ON
//            this.isDebugEnabled = true;
//            if (plotClient == null) {
//                plotClient = new UdpClientPlot(plotClientHost, plotClientPort);
//                if (plotClient.isInitialized()) {
//                    long initTime = System.currentTimeMillis();
//                    plotClient.sendYLimits(initTime, 1.0, 0.0);
//                    plotClient.sendYUnits(initTime + 1, "color & dist/10");
//                }
//            }
//        } else if (!enable && this.isDebugEnabled) {
//            // Turning debugging OFF
//            this.isDebugEnabled = false;
//            close(); // Close the client when disabling
//        }
//    }
//
//    /**
//     * Updates the sensor readings. Should be called in a loop.
//     */
//    public void update() {
//        NormalizedRGBA colors = colorSensor.getNormalizedColors();
//        double distance = (distanceSensor != null) ? distanceSensor.getDistance(DistanceUnit.CM) : -1.0;
//
//        sensorReadings.add(new double[]{colors.red, colors.green, colors.blue, distance});
//
//        if (sensorReadings.size() > ROLLING_WINDOW_SIZE) {
//            sensorReadings.removeFirst();
//        }
//    }
//
//    /**
//     * Analyzes the rolling window to determine the ball color.
//     * @return DetectedColor enum (NONE, GREEN, or PURPLE).
//     */
//    public DetectedColor getBallColor() {
//        // MODIFIED: Check isDebugEnabled for plotting
//        if (sensorReadings.size() < ROLLING_WINDOW_SIZE) {
//            if (isDebugEnabled && plotClient != null) {
//                long time = System.currentTimeMillis();
//                plotClient.sendLineY(time, 0, 1); // avgRed
//                plotClient.sendLineY(time, 0, 2); // avgBlue
//                plotClient.sendLineY(time, 0, 3); // avgGreen
//                plotClient.sendLineY(time, 0, 5); // avgDistance/10
//                plotClient.sendKeyValue(time, "detected_color", "NONE (waiting for data)");
//            }
//            return DetectedColor.NONE;
//        }
//
//        double sumRed = 0, sumGreen = 0, sumBlue = 0, sumDistance = 0;
//        int validDistanceReadings = 0;
//        for (double[] reading : sensorReadings) {
//            sumRed += reading[0];
//            sumGreen += reading[1];
//            sumBlue += reading[2];
//            if (reading[3] >= 0) {
//                sumDistance += reading[3];
//                validDistanceReadings++;
//            }
//        }
//
//        double avgRed = sumRed / ROLLING_WINDOW_SIZE;
//        double avgGreen = sumGreen / ROLLING_WINDOW_SIZE;
//        double avgBlue = sumBlue / ROLLING_WINDOW_SIZE;
//        double avgDistance = (validDistanceReadings > 0) ? (sumDistance / validDistanceReadings) : Double.MAX_VALUE;
//
//        DetectedColor result;
//
//        // --- Detection Logic ---
//        if (distanceSensor == null || avgDistance >= MAX_DETECTION_DISTANCE_CM) {
//            result = DetectedColor.NONE;
//        } else if (avgBlue > avgGreen && avgBlue > avgRed) {
//            result = DetectedColor.PURPLE;
//        } else if (avgGreen > avgRed * 1.2 && avgGreen > avgBlue) {
//            result = DetectedColor.GREEN;
//        } else {
//            result = DetectedColor.NONE;
//        }
//
//        // --- DEBUG Plotting ---
//        // MODIFIED: Check isDebugEnabled
//        if (isDebugEnabled && plotClient != null) {
//            long time = System.currentTimeMillis();
//
//            plotClient.sendLineY(time, avgRed, 1);
//            plotClient.sendLineY(time, avgBlue, 2);
//            plotClient.sendLineY(time, avgGreen, 3);
//
//            double plotDistance = (avgDistance == Double.MAX_VALUE) ? 1.0 : (avgDistance / 10.0);
//            plotClient.sendLineY(time, Math.min(1.0, plotDistance), 5);
//
//            plotClient.sendKeyValue(time, "detected_color", result.toString());
//        }
//
//        return result;
//    }
//
//    /**
//     * Allows setting the gain of the color sensor.
//     */
//    public void setGain(float gain) {
//        colorSensor.setGain(gain);
//    }
//
//    /**
//     * NEW: Closes any open resources, like the UDP client.
//     */
//    public void close() {
//        if (plotClient != null) {
//            plotClient.close();
//            plotClient = null;
//        }
//        this.isDebugEnabled = false;
//    }
//}
