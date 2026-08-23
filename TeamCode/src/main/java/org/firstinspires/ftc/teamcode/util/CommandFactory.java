package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.commands.*;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.ArrayList;
import java.util.List;

/**
 * Factory class to instantiate Command objects from their string names and arguments.
 */
public class CommandFactory {

    /**
     * Creates a Command instance based on the provided class name and arguments.
     *
     * @param className The simple name of the command class.
     * @param argsString The raw arguments string from the mission script (e.g., "[FORWARD]").
     * @param alliance The current alliance color for argument resolution.
     * @return A new Command instance.
     * @throws IllegalArgumentException If the command or arguments are invalid.
     */
    public static Command create(String className, String argsString, Alliance.Color alliance) {
        List<String> args = parseArgs(argsString);

        switch (className) {
            case "IntakeCommand":
                if (args.isEmpty()) throw new IllegalArgumentException("IntakeCommand requires a state argument.");
                Intake.IntakeState state = Intake.IntakeState.valueOf(parseEnumName(args.get(0)));
                return new IntakeCommand(state);

            case "SetInitialPoseCommand":
                if (args.size() < 3) throw new IllegalArgumentException("SetInitialPoseCommand requires [x, y, heading] arguments.");
                double x = Double.parseDouble(args.get(0));
                double y = resolveY(Double.parseDouble(args.get(1)), alliance);
                double heading = Double.parseDouble(args.get(2));
                return new SetInitialPoseCommand(x, y, heading);

            case "WaitCommand":
                if (args.isEmpty()) throw new IllegalArgumentException("WaitCommand requires a duration argument.");
                double waitTime = Double.parseDouble(args.get(0));
                return new WaitCommand(waitTime);

            case "EndCommand":
                return new EndCommand();

            case "WaitForDrivetrainCommand":
                return new WaitForDrivetrainCommand();

            case "WaitForTurretAimCommand":
                double timeout = args.isEmpty() ? 2.0 : Double.parseDouble(args.get(0));
                return new WaitForTurretAimCommand(timeout);

            // Add new commands here as they are implemented in the project.
            // Example for future commands:
            // case "SeekAndAimCommand":
            //     double targetAngle = Double.parseDouble(args.get(0));
            //     Alliance.Color targetAlliance = resolveAlliance(args.get(1), alliance);
            //     return new SeekAndAimCommand(targetAngle, targetAlliance);

            default:
                throw new IllegalArgumentException("Unknown Command class: " + className);
        }
    }

    /**
     * Parses the arguments string into a list of strings.
     * Expected format: "[arg1, arg2, ...]"
     */
    private static List<String> parseArgs(String argsString) {
        List<String> result = new ArrayList<>();
        if (argsString == null || argsString.isEmpty()) return result;

        String clean = argsString.trim();
        if (clean.startsWith("[") && clean.endsWith("]")) {
            clean = clean.substring(1, clean.length() - 1);
        }

        if (clean.isEmpty()) return result;

        String[] parts = clean.split(",");
        for (String part : parts) {
            result.add(part.trim());
        }
        return result;
    }

    /**
     * Resolves an alliance argument, handling the "alliance" keyword.
     */
    private static Alliance.Color resolveAlliance(String arg, Alliance.Color currentAlliance) {
        if ("alliance".equalsIgnoreCase(arg)) {
            return currentAlliance;
        }
        return Alliance.Color.valueOf(parseEnumName(arg));
    }

    /**
     * Mirrors the Y coordinate based on the current alliance.
     * RED alliance uses negative Y, BLUE alliance uses positive Y.
     */
    private static double resolveY(double y, Alliance.Color alliance) {
        return (alliance == Alliance.Color.RED) ? -y : y;
    }

    /**
     * Extracts the enum name from a potentially fully qualified string.
     * E.g., "Intake.IntakeState.FORWARD" -> "FORWARD"
     */
    private static String parseEnumName(String arg) {
        if (arg == null) return null;
        int lastDot = arg.lastIndexOf('.');
        if (lastDot != -1) {
            return arg.substring(lastDot + 1).toUpperCase();
        }
        return arg.toUpperCase();
    }
}
