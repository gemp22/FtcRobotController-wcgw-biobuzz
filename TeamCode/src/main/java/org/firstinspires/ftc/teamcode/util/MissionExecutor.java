package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.commands.*;
import org.firstinspires.ftc.teamcode.pathing.CurvePoint;

import java.util.ArrayList;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Interpreter for the Mission Script.
 * Parses text scripts and populates the CommandScheduler.
 */
public class MissionExecutor {

    private final CommandScheduler scheduler;
    private final Alliance.Color alliance;
    private final boolean isRed;
    private final boolean debug;

    public MissionExecutor(CommandScheduler scheduler, Alliance.Color alliance, boolean debug) {
        this.scheduler = scheduler;
        this.alliance = alliance;
        this.isRed = (alliance == Alliance.Color.RED);
        this.debug = debug;
    }

    /**
     * Parses and executes the script.
     * @param script The full mission script text.
     * @throws MissionParseException If syntax or command errors occur.
     */
    public void execute(String script) throws MissionParseException {
        scheduler.clearAll();

        // 1. Automatic commonInit (mimicking AutoSequenceFactory behavior)
        // Note: Actual commonNearInit implementation details might vary,
        // here we rely on the specific commands to handle their start logic.
        
        String[] lines = script.split("\\r?\\n");
        ArrayList<CurvePoint> currentPathPoints = new ArrayList<>();
        String currentPathMetadata = null;
        double currentPathHeading = 0;

        for (int i = 0; i < lines.length; i++) {
            String line = lines[i].trim();
            int lineNum = i + 1;

            if (line.isEmpty() || line.startsWith("#")) continue;

            try {
                if (line.startsWith("INIT:")) {
                    // Example: INIT: -62.04, 14.00, 0.0
                    // In a real implementation, this might set the robot's initial pose.
                    // For now, we skip it as it's often handled in commonInit.
                } else if (line.startsWith("PATH:")) {
                    // Finish previous path if it exists
                    flushPath(currentPathPoints, currentPathHeading, currentPathMetadata);
                    
                    currentPathPoints = new ArrayList<>();
                    currentPathMetadata = line;
                    currentPathHeading = extractDouble(line, "heading", 0.0);
                } else if (line.startsWith("P:")) {
                    currentPathPoints.add(parsePoint(line, lineNum));
                } else if (line.startsWith("WAIT:")) {
                    flushPath(currentPathPoints, currentPathHeading, currentPathMetadata);
                    currentPathPoints = new ArrayList<>();
                    
                    double time = Double.parseDouble(line.substring(5).trim());
                    WaitCommand waitCmd = new WaitCommand(time);
                    applyMetadata(waitCmd, line);
                    scheduler.add(waitCmd);
                } else if (line.startsWith("CMD:")) {
                    flushPath(currentPathPoints, currentPathHeading, currentPathMetadata);
                    currentPathPoints = new ArrayList<>();

                    String cmdName = extractCmdName(line);
                    String args = extractArgs(line);
                    Command cmd = CommandFactory.create(cmdName, args, alliance);
                    applyMetadata(cmd, line);
                    scheduler.add(cmd);
                } else {
                    throw new MissionParseException("Unknown line type", lineNum);
                }
            } catch (Exception e) {
                if (e instanceof MissionParseException) throw (MissionParseException) e;
                throw new MissionParseException(e.getMessage(), lineNum);
            }
        }
        
        // Final flush
        flushPath(currentPathPoints, currentPathHeading, currentPathMetadata);
        
        // 2. Automatic commonTeardown (mimicking AutoSequenceFactory behavior)
        // In a real implementation, we might add specific teardown commands here.
    }

    private void flushPath(ArrayList<CurvePoint> points, double headingDegrees, String metadata) {
        if (points == null || points.isEmpty()) return;

        FollowPathCommand pathCmd = new FollowPathCommand(points, Math.toRadians(headingDegrees), debug);
        if (metadata != null) {
            applyMetadata(pathCmd, metadata);
        }
        scheduler.add(pathCmd);
    }

    private CurvePoint parsePoint(String line, int lineNum) throws MissionParseException {
        try {
            String[] p = line.substring(2).split(",");
            if (p.length < 7) throw new MissionParseException("Point requires 7 values (x, y, moveSpeed, turnSpeed, followDist, slowRadians, slowAmount)", lineNum);
            
            return new CurvePoint(
                Double.parseDouble(p[0].trim()),
                y(Double.parseDouble(p[1].trim())), // Apply coordinate mirroring
                Double.parseDouble(p[2].trim()),
                Double.parseDouble(p[3].trim()),
                Double.parseDouble(p[4].trim()),
                Math.toRadians(Double.parseDouble(p[5].trim())),
                Double.parseDouble(p[6].trim())
            );
        } catch (NumberFormatException e) {
            throw new MissionParseException("Invalid number format in point", lineNum);
        }
    }

    private double y(double val) {
        return isRed ? -val : val;
    }

    private String extractCmdName(String line) {
        // CMD: IntakeCommand | name="Start"
        int start = 4;
        int end = line.indexOf("|", start);
        if (end == -1) end = line.length();
        return line.substring(start, end).trim();
    }

    private String extractArgs(String line) {
        return extractStringMetadata(line, "args");
    }

    private void applyMetadata(Command cmd, String scriptLine) {
        String name = extractStringMetadata(scriptLine, "name");
        if (name != null && cmd instanceof CommandBase) {
            ((CommandBase) cmd).withName(name);
        }

        if (cmd instanceof FollowPathCommand) {
            FollowPathCommand pathCmd = (FollowPathCommand) cmd;
            if (scriptLine.contains("transition=IMMEDIATE")) {
                pathCmd.transitionImmediately();
            } else if (scriptLine.contains("transition=DIST")) {
                double dist = extractDouble(scriptLine, "transition", -1.0);
                if (dist > 0) {
                    pathCmd.transitionWhenDistancetoEndIsLessThan(dist);
                }
            }
        }
    }

    private String extractStringMetadata(String line, String key) {
        // Matches key="value" or key=[value]
        Pattern pattern = Pattern.compile(key + "=(?:\"([^\"]*)\"|\\[([^\\]]*)\\])");
        Matcher matcher = pattern.matcher(line);
        if (matcher.find()) {
            return matcher.group(1) != null ? matcher.group(1) : matcher.group(2);
        }
        return null;
    }

    private double extractDouble(String line, String key, double defaultValue) {
        // Matches key=123.4 or key(123.4)
        Pattern pattern = Pattern.compile(key + "[=(]([-+]?[0-9]*\\.?[0-9]+)");
        Matcher matcher = pattern.matcher(line);
        if (matcher.find()) {
            String val = matcher.group(1);
            return val != null ? Double.parseDouble(val) : defaultValue;
        }
        return defaultValue;
    }
}
