package org.firstinspires.ftc.teamcode.util;

/**
 * Exception thrown when a mission script fails to parse.
 */
public class MissionParseException extends Exception {
    public MissionParseException(String message) {
        super(message);
    }

    public MissionParseException(String message, int lineNumber) {
        super("Line " + lineNumber + ": " + message);
    }
}
