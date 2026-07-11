package org.firstinspires.ftc.teamcode.util;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.nio.charset.StandardCharsets;
import java.util.Locale;

public class UdpClientFieldSim {
    public static UdpClientFieldSim clientSim;

    private DatagramSocket socket;
    private InetAddress serverAddress;
    private int serverPort;
    private boolean initializedSuccessfully = false; // Flag to track initialization status

    /**
     * Constructs a UDP client to send messages to the Field Simulator.
     * Exceptions during setup are handled internally and printed to System.err.
     *
     * @param serverHost The hostname or IP address of the server (e.g., "localhost").
     * @param serverPort The port number the server is listening on.
     */
    public UdpClientFieldSim(String serverHost, int serverPort) {
        try {
            this.serverAddress = InetAddress.getByName(serverHost);
            this.serverPort = serverPort;
            this.socket = new DatagramSocket(); // OS will assign an available local port
            this.initializedSuccessfully = true;
            System.out.println("UdpClientFieldSim initialized. Target: " + serverHost + ":" + serverPort);
        } catch (UnknownHostException e) {
            System.err.println("UdpClientFieldSim Error: Could not resolve host '" + serverHost + "'. " + e.getMessage());
            // Optionally: e.printStackTrace();
            this.initializedSuccessfully = false;
        } catch (SocketException e) {
            System.err.println("UdpClientFieldSim Error: Could not create UDP socket. " + e.getMessage());
            // Optionally: e.printStackTrace();
            this.initializedSuccessfully = false;
        } catch (SecurityException e) {
            System.err.println("UdpClientFieldSim Error: Security manager denied socket creation. " + e.getMessage());
            this.initializedSuccessfully = false;
        }
    }

    /**
     * Checks if the client was initialized successfully.
     * @return true if initialization succeeded, false otherwise.
     */
    public boolean isInitialized() {
        return initializedSuccessfully;
    }

    /**
     * Sends a raw string message over UDP.
     * This is a generic method; prefer using the type-specific methods for safety.
     *
     * @param message The string message to send.
     */
    private void sendMessage(String message) {
        if (!initializedSuccessfully) {
            System.err.println("UdpClientFieldSim: Client not initialized successfully. Cannot send message: '" + message + "'");
            return;
        }
        if (socket == null || socket.isClosed()) { // Should ideally not happen if initializedSuccessfully is true
            System.err.println("UdpClientFieldSim: Socket is null or closed. Cannot send message: '" + message + "'");
            // Attempt to re-initialize or flag as not initialized? For now, just print error.
            // this.initializedSuccessfully = false; // Could mark as failed here
            return;
        }

        try {
            byte[] buffer = message.getBytes(StandardCharsets.UTF_8);
            DatagramPacket packet = new DatagramPacket(buffer, buffer.length, serverAddress, serverPort);
            socket.send(packet);
            // System.out.println("UdpClientFieldSim sent: '" + message + "'");
        } catch (IOException e) {
            System.err.println("UdpClientFieldSim: Error sending message '" + message + "': " + e.getMessage());
            // Optionally: e.printStackTrace();
        } catch (SecurityException e) {
            System.err.println("UdpClientFieldSim: Security manager denied sending message '" + message + "': " + e.getMessage());
        }
    }

    /**
     * Sends a robot position update.
     * Message format: "pos:x,y,heading"
     *
     * @param x The X coordinate of the robot (in inches).
     * @param y The Y coordinate of the robot (in inches).
     * @param heading The heading of the robot (in degrees).
     */
    public void sendPosition(double x, double y, double heading) {
        if (!isInitialized()) return;
        String message = String.format("pos:%.2f,%.2f,%.1f", x, y, heading);
        sendMessage(message);
    }

    /**
     * Sends a command to draw a circle.
     * Message format: "cir:radiusInches,heading"
     *
     * @param radiusInches The radius of the circle (in inches).
     * @param heading The heading associated with the circle (in degrees).
     */
    public void sendCircle(double radiusInches, double heading) {
        if (!isInitialized()) return;
        String message = String.format("cir:%.2f,%.1f", radiusInches, heading);
        sendMessage(message);
    }

    /**
     * Sends a command to draw a named, styled line segment.
     * Message format: "line:name,x0,y0,x1,y1,styleCode"
     * Style Codes: 1=SOLID_THICK, 2=SOLID_THIN, 3=DOTTED (as defined in UdpPositionListener.LineStyle)
     *
     * @param name A unique string name for this line (e.g., "robot_target", "path_segment_1").
     *             If a line with the same name is sent again, it typically replaces the old one.
     * @param x0 Starting X coordinate (in inches).
     * @param y0 Starting Y coordinate (in inches).
     * @param x1 Ending X coordinate (in inches).
     * @param y1 Ending Y coordinate (in inches).
     * @param styleCode Integer representing the line style (1, 2, or 3).
     */
    public void sendLine(String name, double x0, double y0, double x1, double y1, int styleCode) {
        if (!isInitialized()) return;
        if (name == null || name.trim().isEmpty()) {
            System.err.println("UdpClientFieldSim: Line name cannot be null or empty. Message not sent.");
            return;
        }
        // Basic validation for styleCode, though the receiver (UdpPositionListener) will also handle it
        if (styleCode < 1 || styleCode > 3) { // Assuming 3 styles for now
            System.err.println("UdpClientFieldSim: Invalid styleCode " + styleCode + " for line '" + name + "'. Defaulting or receiver might ignore. Sending anyway.");
            // Or you could choose to not send, or clamp the value: styleCode = Math.max(1, Math.min(3, styleCode));
        }

        // Ensure name does not contain commas, as comma is our delimiter
        if (name.contains(",")) {
            System.err.println("UdpClientFieldSim: Line name '" + name + "' contains a comma. This might break parsing. Replacing commas with underscores.");
            name = name.replace(',', '_');
        }

        String message = String.format(Locale.US, "line:%s,%.2f,%.2f,%.2f,%.2f,%d",
                name, x0, y0, x1, y1, styleCode);
        sendMessage(message);
    }

    /**
     * Sends a command to draw a transient (temporary, often dotted) line segment.
     * Message format: "tline:x0,y0,x1,y1"
     * Only one such line is typically displayed at a time by the simulator.
     *
     * @param x0 Starting X coordinate (in inches).
     * @param y0 Starting Y coordinate (in inches).
     * @param x1 Ending X coordinate (in inches).
     * @param y1 Ending Y coordinate (in inches).
     */
    public void sendTransientLine(double x0, double y0, double x1, double y1) {
        if (!isInitialized()) {
            // System.err.println("UdpClientFieldSim: Not initialized, cannot send transient line."); // Optional
            return;
        }
        String message = String.format("tline:%.2f,%.2f,%.2f,%.2f", x0, y0, x1, y1);
        sendMessage(message);
    }

    /**
     * Sends an arbitrary text message.
     * Message format: "txt:your text message here"
     *
     * @param text The text message to send.
     */
    public void sendText(String text) {
        if (!isInitialized()) return;
        String message = "txt:" + text;
        sendMessage(message);
    }

    /**
     * Sends a key-value pair to be displayed in the simulator's table.
     * Message format: "kv:key,value"
     *
     * @param key   The key (identifier) for the data.
     * @param value The string representation of the value.
     */
    public void sendKeyValue(String key, String value) {
        if (!isInitialized()) return;
        if (key == null || key.trim().isEmpty()) {
            System.err.println("UdpClientFieldSim: Key cannot be null or empty. Message not sent.");
            return;
        }
        // Ensure key/value do not contain commas to avoid breaking the simple parser.
        // A more robust format would use quoting or a different delimiter.
        if (key.contains(",")) {
            System.err.println("UdpClientFieldSim: Key '" + key + "' contains a comma. Replacing with underscore.");
            key = key.replace(',', '_');
        }
        if (value != null && value.contains(",")) {
            System.err.println("UdpClientFieldSim: Value '" + value + "' contains a comma. Replacing with semicolon.");
            value = value.replace(',', ';'); // Use a different character to avoid confusion
        }

        String message = String.format("kv:%s,%s", key, value != null ? value : "");
        sendMessage(message);
    }

    /**
     * Closes the UDP socket.
     * Should be called when the client is no longer needed to release resources.
     */
    public void close() {
        if (socket != null && !socket.isClosed()) {
            socket.close();
            System.out.println("UdpClientFieldSim socket closed.");
        }
        // No real exceptions to handle here for close that aren't already covered by DatagramSocket itself.
    }

    /**
     * Example usage of the UdpClientFieldSim.
     */
    public static void main(String[] args) {
        String host = "localhost";
        int port = 7777;
        UdpClientFieldSim client = new UdpClientFieldSim(host, port);

        if (!client.isInitialized()) {
            System.err.println("Main: UdpClientFieldSim failed to initialize. Exiting example.");
            return;
        }

        try {
            client.sendText("Client connected! Testing new line formats.");
            Thread.sleep(500);

            client.sendKeyValue("Robot Status", "Initializing");
            client.sendKeyValue("Target Lock", "False");
            client.sendKeyValue("Alliance", "BLUE");
            Thread.sleep(500);

            // ... later in the main method, for example after a position update
            client.sendPosition(10.0, 20.0, 45.0);
            client.sendKeyValue("Robot Status", "Moving to Target");
            Thread.sleep(500);

            // ... later, update a value
            client.sendKeyValue("Target Lock", "True");
            Thread.sleep(500);

            client.sendPosition(10.0, 20.0, 45.0);
            Thread.sleep(500);

            // Test new line formats
            client.sendLine("target_vector", 10.0, 20.0, 50.0, 60.0, 3); // Dotted line from robot
            Thread.sleep(1000);

            client.sendLine("wall_boundary_1", -70.0, -70.0, 70.0, -70.0, 1); // Thick solid line
            Thread.sleep(500);
            client.sendLine("wall_boundary_2", 70.0, -70.0, 70.0, 70.0, 1);  // Thick solid line
            Thread.sleep(500);

            client.sendCircle(15.0, 90.0);
            Thread.sleep(500);

            client.sendLine("center_cross_horizontal", -20.0, 0.0, 20.0, 0.0, 2); // Thin solid line
            Thread.sleep(500);
            client.sendLine("center_cross_vertical", 0.0, -20.0, 0.0, 20.0, 2);   // Thin solid line
            Thread.sleep(1000);

            // Update a line by using the same name
            client.sendText("Updating target_vector to new position/style");
            client.sendLine("target_vector", 10.0, 20.0, 0.0, 0.0, 2); // Now a thin solid line to origin
            Thread.sleep(1000);

            client.sendPosition(-10.0, -5.0, 180.0);
            Thread.sleep(500);

            client.sendLine("another_dotted", -10.0, -5.0, -50.0, -30.0, 3);
            Thread.sleep(1000);

            client.sendText("Lines drawn. Test complete.");

        } catch (InterruptedException e) {
            System.err.println("Main: Thread interrupted during example sequence: " + e.getMessage());
            Thread.currentThread().interrupt();
        } finally {
            client.close();
        }

        System.out.println("\n--- Testing with potentially invalid host ---");
        UdpClientFieldSim badClient = new UdpClientFieldSim("nonexistenthost123abc", port);
        if (badClient.isInitialized()) {
            badClient.sendText("This should not send if host was bad.");
            badClient.close();
        } else {
            System.out.println("Main: Bad client correctly reported as not initialized.");
        }
    }
}
