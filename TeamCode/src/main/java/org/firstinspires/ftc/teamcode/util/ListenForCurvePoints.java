package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.pathing.CurvePoint;
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.net.SocketTimeoutException;
import java.util.ArrayList;

/**
 * A singleton utility class that listens for robot start pose, follow angle, and CurvePoint data sent over UDP.
 * Designed to run on a background thread for non-blocking operation.
 */
public class ListenForCurvePoints {

    // --- Configuration ---
    private static final int UDP_PORT = 6666;
    private static final String FOLLOW_ANGLE_PREFIX = "follow_angle:"; // NEW
    private static final String START_ROBOT_POS_PREFIX = "start_robot_pos:";
    private static final String CURVE_POINT_PREFIX = "curve_point:";
    private static final String END_MESSAGE = "end";
    private static final int BUFFER_SIZE = 1024;
    public static final int TIMEOUT_MILLISECONDS = 30000; // 30 seconds for activity timeout
    private static final int SOCKET_RECEIVE_TIMEOUT_MS = 500; // Shorter timeout for responsive loop

    // --- Singleton Implementation ---
    private static volatile ListenForCurvePoints instance;

    private ListenForCurvePoints() { /* Private constructor */ }

    public static ListenForCurvePoints getInstance() {
        if (instance == null) {
            synchronized (ListenForCurvePoints.class) {
                if (instance == null) {
                    instance = new ListenForCurvePoints();
                }
            }
        }
        return instance;
    }

    // --- State Variables ---
    private volatile ListenStatus currentStatus = ListenStatus.NOT_STARTED;
    private final ArrayList<CurvePoint> collectedPoints = new ArrayList<>();
    private volatile double startPoseX = Double.NaN;
    private volatile double startPoseY = Double.NaN;
    private volatile double startPoseHeading = Double.NaN;
    private volatile double followAngle = 90.0; // NEW: Default to 90 degrees
    private volatile String internalErrorMessage = null;
    private volatile long lastPacketReceivedTimeMs;

    private Thread listenerThread;
    private DatagramSocket socket;

    public synchronized void startListening() {
        if (currentStatus == ListenStatus.LISTENING) {
            System.out.println("Listener: Already active. Stopping previous and starting new.");
            stopListeningInternal(true);
        }

        // Reset all state for the new session
        collectedPoints.clear();
        startPoseX = Double.NaN;
        startPoseY = Double.NaN;
        startPoseHeading = Double.NaN;
        followAngle = 90.0; // NEW: Reset to default
        internalErrorMessage = null;
        currentStatus = ListenStatus.LISTENING;
        lastPacketReceivedTimeMs = System.currentTimeMillis();

        listenerThread = new Thread(this::runListeningLoop, "CurvePointListenerThread");
        listenerThread.setDaemon(true);
        listenerThread.start();
        System.out.println("Listener: Curve point listener thread started.");
    }

    private void runListeningLoop() {
        DatagramSocket localSocket = null;
        try {
            localSocket = new DatagramSocket(UDP_PORT);
            localSocket.setSoTimeout(SOCKET_RECEIVE_TIMEOUT_MS);
            this.socket = localSocket;

            System.out.println("Listener: Listening on UDP port: " + UDP_PORT);

            byte[] buffer = new byte[BUFFER_SIZE];
            while (currentStatus == ListenStatus.LISTENING) {
                if (Thread.currentThread().isInterrupted()) {
                    currentStatus = ListenStatus.STOPPED;
                    break;
                }

                if (System.currentTimeMillis() - lastPacketReceivedTimeMs > TIMEOUT_MILLISECONDS) {
                    currentStatus = ListenStatus.TIMED_OUT;
                    break;
                }

                DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
                try {
                    localSocket.receive(packet);
                    lastPacketReceivedTimeMs = System.currentTimeMillis();
                    String rawMessage = new String(packet.getData(), 0, packet.getLength()).trim();

                    if (END_MESSAGE.equalsIgnoreCase(rawMessage)) {
                        currentStatus = ListenStatus.COMPLETED_SUCCESSFULLY;
                        break;
                    } else if (rawMessage.startsWith(FOLLOW_ANGLE_PREFIX)) { // NEW
                        parseFollowAngleMessage(rawMessage);
                    } else if (rawMessage.startsWith(START_ROBOT_POS_PREFIX)) {
                        parseStartPoseMessage(rawMessage);
                    } else if (rawMessage.startsWith(CURVE_POINT_PREFIX)) {
                        CurvePoint point = parseCurvePointMessage(rawMessage);
                        if (point != null) {
                            synchronized (collectedPoints) {
                                collectedPoints.add(point);
                            }
                        }
                    }
                } catch (SocketTimeoutException e) {
                    // Expected, allows loop to check status
                } catch (IOException e) {
                    if (currentStatus == ListenStatus.LISTENING) {
                        internalErrorMessage = "IOException: " + e.getMessage();
                        currentStatus = ListenStatus.ERROR;
                    }
                    break;
                }
            }
        } catch (SocketException e) {
            if (currentStatus == ListenStatus.LISTENING) {
                internalErrorMessage = "SocketException: " + e.getMessage();
                currentStatus = ListenStatus.ERROR;
            }
        } finally {
            this.socket = null;
            if (localSocket != null && !localSocket.isClosed()) {
                localSocket.close();
            }
            if (currentStatus == ListenStatus.LISTENING) {
                currentStatus = ListenStatus.STOPPED;
            }
            System.out.println("Listener: Loop finished with status: " + currentStatus);
        }
    }

    public ListenResult checkStatusAndGetPoints() {
        ListenStatus statusSnapshot;
        ArrayList<CurvePoint> pointsSnapshot;
        String errorSnapshot;
        double startXSnapshot, startYSnapshot, startHeadingSnapshot, followAngleSnapshot;

        synchronized (this) {
            statusSnapshot = this.currentStatus;
            errorSnapshot = this.internalErrorMessage;
            startXSnapshot = this.startPoseX;
            startYSnapshot = this.startPoseY;
            startHeadingSnapshot = this.startPoseHeading;
            followAngleSnapshot = this.followAngle; // NEW
            synchronized (collectedPoints) {
                pointsSnapshot = new ArrayList<>(this.collectedPoints);
            }
        }
        // Pass the new value to the updated constructor
        return new ListenResult(statusSnapshot, pointsSnapshot, errorSnapshot, startXSnapshot, startYSnapshot, startHeadingSnapshot, followAngleSnapshot);
    }

    public synchronized void stopListening() {
        stopListeningInternal(false);
    }

    private void stopListeningInternal(boolean internalCall) {
        if (currentStatus == ListenStatus.LISTENING || (listenerThread != null && listenerThread.isAlive())) {
            currentStatus = ListenStatus.STOPPED;

            DatagramSocket s = this.socket;
            if (s != null && !s.isClosed()) {
                s.close();
            }

            if (listenerThread != null && listenerThread.isAlive()) {
                listenerThread.interrupt();
                try {
                    listenerThread.join(SOCKET_RECEIVE_TIMEOUT_MS + 200);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        }
        listenerThread = null;
        this.socket = null;
    }

    // --- NEW: Parser for the follow angle message ---
    private void parseFollowAngleMessage(String message) {
        String content = message.substring(FOLLOW_ANGLE_PREFIX.length());
        try {
            this.followAngle = Double.parseDouble(content.trim());
            System.out.printf("Listener Parser: Received Follow Angle -> %.2f deg%n", followAngle);
        } catch (NumberFormatException e) {
            System.err.println("Listener Parser: Malformed follow angle data. Error: " + e.getMessage());
        }
    }

    private void parseStartPoseMessage(String message) {
        String content = message.substring(START_ROBOT_POS_PREFIX.length());
        String[] parts = content.split(",");
        if (parts.length == 3) {
            try {
                this.startPoseX = Double.parseDouble(parts[0].trim());
                this.startPoseY = Double.parseDouble(parts[1].trim());
                this.startPoseHeading = Double.parseDouble(parts[2].trim());
                System.out.printf("Listener Parser: Received Start Pose -> X: %.2f, Y: %.2f, H: %.2f%n", startPoseX, startPoseY, startPoseHeading);
            } catch (NumberFormatException e) {
                System.err.println("Listener Parser: Malformed start pose data. Error: " + e.getMessage());
            }
        }
    }

    private CurvePoint parseCurvePointMessage(String message) {
        String content = message.substring(CURVE_POINT_PREFIX.length());
        String[] parts = content.split(",");
        if (parts.length == 7) {
            try {
                double x = Double.parseDouble(parts[0].trim());
                double y = Double.parseDouble(parts[1].trim());
                double moveSpeed = Double.parseDouble(parts[2].trim());
                double turnSpeed = Double.parseDouble(parts[3].trim());
                double followDistance = Double.parseDouble(parts[4].trim());
                double slowDownTurnRadians = Double.parseDouble(parts[5].trim());
                double slowDownTurnAmount = Double.parseDouble(parts[6].trim());
                return new CurvePoint(x, y, moveSpeed, turnSpeed, followDistance, slowDownTurnRadians, slowDownTurnAmount);
            } catch (NumberFormatException e) {
                System.err.println("Listener Parser: Malformed curve point data. Error: " + e.getMessage());
                return null;
            }
        }
        return null;
    }
}
