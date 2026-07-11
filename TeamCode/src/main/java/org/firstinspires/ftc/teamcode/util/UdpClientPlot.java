package org.firstinspires.ftc.teamcode.util;

// In the robot's teamcode package, or a shared library


import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.nio.charset.StandardCharsets;
import java.util.Locale;

/**
 * Utility class for robot code to send UDP messages to the FtcFieldSimulator's PlotDisplay.
 * This class would typically reside in the FTC robot controller's `teamcode` module or a shared library.
 */
public class UdpClientPlot {
    private DatagramSocket socket;
    private InetAddress address;
    private int port;
    private boolean initialized = false;

    private static final String DEFAULT_SIMULATOR_IP = "127.0.0.1";

    /**
     * Constructor for UdpClientPlot.
     * @param host The hostname or IP address of the machine running the FtcFieldSimulator.
     * @param port The port number the UdpPlotListener in FtcFieldSimulator is listening on.
     */
    public UdpClientPlot(String host, int port) {
        try {
            this.socket = new DatagramSocket();
            this.address = InetAddress.getByName(host);
            this.port = port;
            this.initialized = true;
            System.out.println("UdpClientPlot initialized to send to " + host + ":" + port);
        } catch (Exception e) {
            System.err.println("UdpClientPlot: Failed to initialize UDP client for " + host + ":" + port + " - " + e.getMessage());
            this.initialized = false;
        }
    }

    /**
     * Default constructor. Attempts to connect to the simulator on localhost.
     * NOTE: Requires a known port, so you should specify it.
     * This constructor is less ideal now; use the one with host and port.
     */
    public UdpClientPlot() {
        // This port needs to be known. Assuming a default if not specified.
        this(DEFAULT_SIMULATOR_IP, 7778); // Example default port
    }

    private void sendMessage(String message) {
        if (!initialized || socket == null || socket.isClosed()) {
            return;
        }
        try {
            byte[] buffer = message.getBytes(StandardCharsets.UTF_8);
            DatagramPacket packet = new DatagramPacket(buffer, buffer.length, address, port);
            socket.send(packet);
        } catch (IOException e) {
            System.err.println("UdpClientPlot: IOException sending message '" + message + "': " + e.getMessage());
        } catch (Exception e) {
            System.err.println("UdpClientPlot: Exception sending message '" + message + "': " + e.getMessage());
        }
    }

    // --- Primary Y-Axis (Left) Methods ---

    /**
     * Sends a discrete data point to be plotted on the primary (left) Y-axis.
     */
    public void sendPointY(long timestamp, double yValue, int style) {
        String message = String.format(Locale.US, "POINT %d %d %.6f",
                timestamp, style, yValue);
        sendMessage(message);
    }

    /**
     * Sends a data point for a line series on the primary (left) Y-axis.
     */
    public void sendLineY(long timestamp, double yValue, int style) {
        String message = String.format(Locale.US, "LINE %d %d %.6f",
                timestamp, style, yValue);
        sendMessage(message);
    }

    /**
     * Sets the Y-axis limits for the primary (left) axis.
     */
    public void sendYLimits(long timestamp, double maxY, double minY) {
        String message = String.format(Locale.US, "YLIMITS %d %.6f %.6f",
                timestamp, minY, maxY); // Note: Swapped to minY, maxY to match PlotDisplay loader
        sendMessage(message);
    }

    /**
     * Sets the unit label for the primary (left) Y-axis.
     */
    public void sendYUnits(long timestamp, String unitString) {
        String message = String.format(Locale.US, "YUNITS %d \"%s\"",
                timestamp, unitString);
        sendMessage(message);
    }


    // --- NEW: Secondary Y-Axis (Right) Methods ---

    /**
     * Sends a discrete data point to be plotted on the secondary (right) Y-axis.
     */
    public void sendPointY2(long timestamp, double yValue, int style) {
        String message = String.format(Locale.US, "POINT2 %d %d %.6f",
                timestamp, style, yValue);
        sendMessage(message);
    }

    /**
     * Sends a data point for a line series on the secondary (right) Y-axis.
     */
    public void sendLineY2(long timestamp, double yValue, int style) {
        String message = String.format(Locale.US, "LINE2 %d %d %.6f",
                timestamp, style, yValue);
        sendMessage(message);
    }

    /**
     * Sets the Y-axis limits for the secondary (right) axis.
     */
    public void sendYLimits2(long timestamp, double maxY, double minY) {
        String message = String.format(Locale.US, "YLIMITS2 %d %.6f %.6f",
                timestamp, minY, maxY);
        sendMessage(message);
    }

    /**
     * Sets the unit label for the secondary (right) Y-axis.
     */
    public void sendYUnits2(long timestamp, String unitString) {
        String message = String.format(Locale.US, "YUNITS2 %d \"%s\"",
                timestamp, unitString);
        sendMessage(message);
    }


    // --- Shared Methods (Unaffected by multiple axes) ---

    /**
     * Sends a text annotation to be displayed as a vertical marker on the plot.
     */
    public void sendTextMarker(long timestamp, String text, String positionKeyword) {
        String message = String.format(Locale.US, "MARKER %d %s \"%s\"",
                timestamp, positionKeyword.toLowerCase(Locale.US), text);
        sendMessage(message);
    }

    /**
     * Sends a key-value pair to be displayed in the data table.
     */
    public void sendKeyValue(long timestamp, String key, String value) {
        String message = String.format(Locale.US, "KV %d \"%s\" \"%s\"",
                timestamp, key, value);
        sendMessage(message);
    }


    /**
     * Assigns a name to a line series on the primary (left) Y-axis.
     * @param timestamp The current time.
     * @param label The name for the series (e.g., "Target RPM").
     * @param style The style index (1-10) this name applies to.
     */
    public void sendSeriesNameLine(long timestamp, String label, int style) {
        String message = String.format(Locale.US, "SERIESNAMELINE %d \"%s\" %d",
                timestamp, label, style);
        sendMessage(message);
    }

    /**
     * Assigns a name to a point series on the primary (left) Y-axis.
     * @param timestamp The current time.
     * @param label The name for the series.
     * @param style The style index (1-10) this name applies to.
     */
    public void sendSeriesNamePoint(long timestamp, String label, int style) {
        String message = String.format(Locale.US, "SERIESNAMEPOINT %d \"%s\" %d",
                timestamp, label, style);
        sendMessage(message);
    }

    /**
     * Assigns a name to a line series on the secondary (right) Y-axis.
     * @param timestamp The current time.
     * @param label The name for the series (e.g., "Motor Power").
     * @param style The style index (1-10) this name applies to.
     */
    public void sendSeriesNameLine2(long timestamp, String label, int style) {
        String message = String.format(Locale.US, "SERIESNAMELINE2 %d \"%s\" %d",
                timestamp, label, style);
        sendMessage(message);
    }

    /**
     * Assigns a name to a point series on the secondary (right) Y-axis.
     * @param timestamp The current time.
     * @param label The name for the series.
     * @param style The style index (1-10) this name applies to.
     */
    public void sendSeriesNamePoint2(long timestamp, String label, int style) {
        String message = String.format(Locale.US, "SERIESNAMEPOINT2 %d \"%s\" %d",
                timestamp, label, style);
        sendMessage(message);
    }

    // --- Client Management ---

    public boolean isInitialized() {
        return initialized;
    }

    public void close() {
        if (socket != null && !socket.isClosed()) {
            socket.close();
            System.out.println("UdpClientPlot socket closed.");
        }
        initialized = false;
    }

    // Optional: A main method for quick testing from a non-robot environment
    public static void main(String[] args) {
        // Test sending to a specific port
        UdpClientPlot plotClient = new UdpClientPlot("127.0.0.1", 7778);
        if (!plotClient.isInitialized()) {
            System.err.println("Plot client failed to initialize. Exiting test.");
            return;
        }
        long startTime = System.currentTimeMillis();

        // --- Setup both Y-Axes ---
        plotClient.sendYUnits(System.currentTimeMillis() - startTime, "RPM");
        plotClient.sendYLimits(System.currentTimeMillis() - startTime, 6000, 0);

        plotClient.sendYUnits2(System.currentTimeMillis() - startTime, "Power");
        plotClient.sendYLimits2(System.currentTimeMillis() - startTime, 1.0, -1.0);


        for (int i = 0; i < 100; i++) {
            long time = System.currentTimeMillis() - startTime;
            // Data for Left Y-Axis (RPM)
            double rpmValue = 3000 + 2500 * Math.sin(i * 0.1);
            plotClient.sendLineY(time, rpmValue, 1); // Style 1 for RPM

            // Data for Right Y-Axis (Power)
            double powerValue = 0.7 * Math.cos(i * 0.1);
            plotClient.sendLineY2(time, powerValue, 2); // Style 2 for Power

            if (i % 20 == 0) {
                plotClient.sendTextMarker(time, "Event " + i, "mid");
                plotClient.sendKeyValue(time, "Loop", String.valueOf(i));
            }
            try { Thread.sleep(50); } catch (InterruptedException e) { break; }
        }
        plotClient.sendKeyValue(System.currentTimeMillis() - startTime, "Status", "Test Complete");
        plotClient.close();
    }
}

