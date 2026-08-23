package org.firstinspires.ftc.teamcode.util;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.net.SocketTimeoutException;

/**
 * Robot-side listener for the Mission Script received via UDP.
 * This class follows the singleton pattern to ensure only one listener is active.
 */
public class MissionListener {
    private static final int UDP_PORT = 6666;
    private static final String START_MARKER = "MISSION_START";
    private static final String END_MARKER = "MISSION_END";
    
    private static volatile MissionListener instance;
    private StringBuilder scriptBuffer = new StringBuilder();
    private boolean isBuffering = false;
    private String lastReceivedScript = null;
    private boolean hasNewScript = false;
    private Thread listenThread;

    private MissionListener() {}

    public static MissionListener getInstance() {
        if (instance == null) {
            synchronized (MissionListener.class) {
                if (instance == null) instance = new MissionListener();
            }
        }
        return instance;
    }

    public synchronized boolean hasNewMission() {
        return hasNewScript;
    }
    
    public synchronized String getMissionAndReset() {
        hasNewScript = false;
        String script = lastReceivedScript;
        lastReceivedScript = null;
        return script;
    }

    public synchronized void startListening() {
        if (listenThread != null && listenThread.isAlive()) {
            return; // Already listening
        }
        listenThread = new Thread(this::runLoop);
        listenThread.setName("MissionListenerThread");
        listenThread.start();
    }

    public synchronized void stopListening() {
        if (listenThread != null) {
            listenThread.interrupt();
            listenThread = null;
        }
    }

    private void runLoop() {
        try (DatagramSocket socket = new DatagramSocket(UDP_PORT)) {
            socket.setSoTimeout(1000);
            byte[] buffer = new byte[1024];
            
            while (!Thread.currentThread().isInterrupted()) {
                try {
                    DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
                    socket.receive(packet);
                    String msg = new String(packet.getData(), 0, packet.getLength());

                    if (msg.contains(START_MARKER)) {
                        scriptBuffer = new StringBuilder();
                        isBuffering = true;
                    } else if (msg.contains(END_MARKER)) {
                        synchronized (this) {
                            lastReceivedScript = scriptBuffer.toString();
                            hasNewScript = true;
                        }
                        isBuffering = false;
                    } else if (isBuffering) {
                        scriptBuffer.append(msg);
                    }
                } catch (SocketTimeoutException ignored) {
                    // Normal timeout for check of interruption
                } catch (IOException e) {
                    System.err.println("MissionListener IO Error: " + e.getMessage());
                    break;
                }
            }
        } catch (SocketException e) {
            System.err.println("MissionListener Socket Error: " + e.getMessage());
        }
    }
}
