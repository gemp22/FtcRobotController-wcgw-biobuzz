package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.MissionExecutor;
import org.firstinspires.ftc.teamcode.util.MissionListener;
import org.firstinspires.ftc.teamcode.util.MissionParseException;

/**
 * TeleOp OpMode to receive and execute Mission Scripts from FTCSimulator.
 */
@TeleOp(name = "Mission Runner", group = "Main")
public class TeleOp_MissionRunner extends OpMode {

    private RobotHardware robot;
    private Drivetrain drivetrain;
    private Intake intake;
    private CommandScheduler scheduler;
    private MissionListener listener;
    private MissionExecutor executor;

    private Alliance.Color selectedAlliance = Alliance.Color.BLUE;
    private boolean debugMode = true;
    private String lastError = null;
    private boolean isExecuting = false;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        robot = new RobotHardware();
        robot.init(hardwareMap);

        drivetrain = new Drivetrain(robot);
        drivetrain.setDebug(true);
        intake = new Intake(robot);
        scheduler = new CommandScheduler(drivetrain, intake);
        scheduler.setTeleOpMode(true); // Disable auto deadline for tuning/testing

        listener = MissionListener.getInstance();
        listener.startListening();

        telemetry.addData("Status", "Ready. Select Alliance and wait for Mission.");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Alliance Selection
        if (gamepad1.x) {
            selectedAlliance = Alliance.Color.BLUE;
        } else if (gamepad1.b) {
            selectedAlliance = Alliance.Color.RED;
        }

        // Debug Toggle
        if (gamepad1.y) {
            debugMode = !debugMode;
        }

        telemetry.addLine("=== MISSION RUNNER CONFIG ===");
        telemetry.addData("Alliance (X: Blue, B: Red)", selectedAlliance);
        telemetry.addData("Debug Mode (Y)", debugMode ? "ENABLED" : "DISABLED");
        telemetry.addLine("-----------------------------");
        
        if (listener.hasNewMission()) {
            telemetry.addLine("NEW MISSION DETECTED!");
        } else {
            telemetry.addLine("Waiting for mission from simulator (press 'Send Mission to Robot')...");
        }
        telemetry.update();
    }

    @Override
    public void start() {
        executor = new MissionExecutor(scheduler, selectedAlliance, debugMode);
    }

    @Override
    public void loop() {
        // Check for new mission
        if (listener.hasNewMission()) {
            String script = listener.getMissionAndReset();
            try {
                executor.execute(script);
                lastError = null;
                isExecuting = true;
            } catch (MissionParseException e) {
                lastError = e.getMessage();
                isExecuting = false;
                scheduler.clearAll();
            }
        }

        // Update Subsystems
        for (LynxModule module : robot.allHubs) {
            module.clearBulkCache();
        }
        
        drivetrain.update();
        intake.update();
        scheduler.update();

        updateTelemetry();
    }

    @Override
    public void stop() {
        if (listener != null) {
            listener.stopListening();
        }
        if (scheduler != null) {
            scheduler.clearAll();
        }
    }

    private void updateTelemetry() {
        telemetry.addLine("=== MISSION RUNNER STATUS ===");
        telemetry.addData("Alliance", selectedAlliance);
        
        if (lastError != null) {
            telemetry.addLine("!!! PARSE ERROR !!!");
            telemetry.addLine(lastError);
        } else if (isExecuting) {
            if (scheduler.isBusy()) {
                telemetry.addData("Executing", scheduler.getCurrentCommandName());
            } else {
                telemetry.addLine("Mission Finished.");
                isExecuting = false;
            }
        } else {
            telemetry.addLine("Idle. Waiting for new mission.");
        }
        
        telemetry.addLine("-----------------------------");
        telemetry.addData("Robot Pose", drivetrain.getPose().toString());
        telemetry.update();
    }
}
