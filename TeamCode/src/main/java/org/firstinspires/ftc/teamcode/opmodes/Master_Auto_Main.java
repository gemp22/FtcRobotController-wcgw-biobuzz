package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.commands.CommandBase;
import org.firstinspires.ftc.teamcode.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.Alliance;

import java.util.Locale;

/**
 * Master Autonomous Manager.
 * Detects Alliance, Position, and Pattern automatically.
 * Press 'A' to toggle Manual Override for Alliance and Position.
 */
@Autonomous(name = "Master Auto Manager", group = "Main")
public class Master_Auto_Main extends OpMode {

    // --- Subsystems ---
    private RobotHardware robot;
    private Drivetrain drivetrain;
    private Intake intake;
//    private Shooter shooter;
//    private Turret turret;
    private CommandScheduler scheduler;

    // --- Selection State ---
    private boolean isManualOverride = false;
    private int manualConfigIndex = 0; // 0 to 19 (Pos x Alliance x Variant)
    private int selectedVariantIndex = 0; // Used for Auto-Mode only
    private final String[] variantLabels = {"A", "B", "C", "D", "E"};
    private static final String[] FAR_TITLES = {"Corner Husky", "Corner & Spike", "BIOBUZZ Test Auto", "BioBuzz Test Auto V2", "Corner Husky Only"};
    private static final String[] NEAR_TITLES = {"Double Gate", "Open Gate", "Sort", "TBD", "TBD"};

    private double startDelay = 0.0;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing Master Manager...");

        robot = new RobotHardware();
        robot.init(hardwareMap);

        drivetrain = new Drivetrain(robot);
        intake = new Intake(robot);
//        shooter = new Shooter(robot);
//        turret = new Turret(robot);

        drivetrain.resetOdometry();
        scheduler = new CommandScheduler(drivetrain, intake, 29.0);

//        Turret.resetGamePattern();
//        turret.startAutoDetectStart();
//        turret.setInitialPosition(0);

        startDelay = 0.0;
        isManualOverride = false;
        manualConfigIndex = 0;

        telemetry.update();
    }

    @Override
    public void init_loop() {
        for (LynxModule module : robot.allHubs) {
            module.clearBulkCache();
        }

//        turret.update(drivetrain);

        // Toggle Manual Override
        if (gamepad1.aWasPressed()) {
            isManualOverride = !isManualOverride;
            if (isManualOverride) manualConfigIndex = 0; // Start at index 0 as requested
        }

        if (isManualOverride) {
            // Handle scrolling through all 20 options
            if (gamepad1.dpadDownWasPressed()) manualConfigIndex = (manualConfigIndex + 1) % 20;
            if (gamepad1.dpadUpWasPressed()) manualConfigIndex = (manualConfigIndex + 19) % 20;
        } else {
            // Handle Variant Selection (Auto Mode Only)
            if (gamepad1.dpadDownWasPressed()) selectedVariantIndex = (selectedVariantIndex + 1) % 5;
            if (gamepad1.dpadUpWasPressed()) selectedVariantIndex = (selectedVariantIndex + 4) % 5;
        }

        // Handle Start Delay (Horizontal DPAD)
        if (gamepad1.dpadRightWasPressed()) startDelay += 2.0;
        if (gamepad1.dpadLeftWasPressed()) startDelay = Math.max(0, startDelay - 2.0);

        displayUI();
    }

    @Override
    public void start() {
        Alliance.Color alliance;
        Alliance.Position position;
        int variant;

        if (isManualOverride) {
            position = (manualConfigIndex < 10) ? Alliance.Position.NEAR : Alliance.Position.FAR;
            alliance = ((manualConfigIndex / 5) % 2 == 0) ? Alliance.Color.BLUE : Alliance.Color.RED;
            variant = manualConfigIndex % 5;
        } else {
            alliance = Alliance.Color.BLUE; // Change To Auto Detect
            if (alliance == null) alliance = Alliance.Color.BLUE;

            position = Alliance.Position.FAR;
            if (position == Alliance.Position.UNKNOWN) position = Alliance.Position.FAR;

            variant = selectedVariantIndex;
        }

        // Pattern detection is always used (even in override)
//        if (Turret.getDecodedGamePattern() == Turret.DecodePattern.UNKNOWN) {
//            Turret.forceSetGamePattern(Turret.DecodePattern.PPG);
//        }

        System.out.printf("MASTER START: Mode=%s, Alliance=%s, Pos=%s, Variant=%s, Delay=%.1fs%n",
                isManualOverride ? "MANUAL" : "AUTO", alliance, position, variantLabels[variant], startDelay);

        buildAutonomousSequence(alliance, position, variant, startDelay);
//        turret.aimAtTag(alliance);
    }

    @Override
    public void loop() {
        for (LynxModule module : robot.allHubs) {
            module.clearBulkCache();
        }
        drivetrain.update();
        intake.update();
//        shooter.update(intake, turret, drivetrain);
//        turret.update(drivetrain);
        scheduler.update();
        updateTelemetry();
    }

    @Override
    public void stop() {
        CommandBase.saveAutoState(drivetrain);
        if (scheduler != null) scheduler.reset();
        super.stop();
    }

    private void displayUI() {
        if (isManualOverride) {
            telemetry.addLine("=== MANUAL OVERRIDE ACTIVE ===");
            telemetry.addLine("DPAD Up/Down: Select Config | 'A': Back to Auto");
            telemetry.addLine("------------------------------------------");

            // Show a scrolling window of the 20 options
            int start = Math.max(0, manualConfigIndex - 2);
            int end = Math.min(19, start + 5);
            for (int i = start; i <= end; i++) {
                String label = getOptionLabel(i);
                if (i == manualConfigIndex) {
                    telemetry.addLine(" > " + label + " <");
                } else {
                    telemetry.addLine("   " + label);
                }
            }
        } else {
            telemetry.addLine("=== AUTO DETECTION MODE ===");
            telemetry.addLine("'A': Toggle Manual Override");

            Alliance.Color detectedAll = Alliance.Color.BLUE;
            Alliance.Position detectedPos = Alliance.Position.FAR;

            telemetry.addData("Alliance", detectedAll == null ? "DETECTING..." : detectedAll);
            telemetry.addData("Position", detectedPos == Alliance.Position.UNKNOWN ? "DETECTING..." : detectedPos);
            telemetry.addData("Selected", variantLabels[selectedVariantIndex]);

            // Restore the Variant Legend so the user knows what A-E do
            telemetry.addLine("\n=== VARIANT SELECTION (D-Pad) ===");
            String[] titles = (detectedPos == Alliance.Position.FAR) ? FAR_TITLES : NEAR_TITLES;

            for (int i = 0; i < 5; i++) {
                String prefix = (selectedVariantIndex == i) ? " > [*] " : "    [  ] ";
                telemetry.addLine(prefix + variantLabels[i] + ": " + titles[i]);
            }
        }

//        telemetry.addData("Pattern", Turret.getDecodedGamePattern());
        telemetry.addData("Start Delay", "%.1fs", startDelay);
        telemetry.update();
    }

    private String getOptionLabel(int index) {
        Alliance.Position pos = (index < 10) ? Alliance.Position.NEAR : Alliance.Position.FAR;
        Alliance.Color all = ((index / 5) % 2 == 0) ? Alliance.Color.BLUE : Alliance.Color.RED;
        int varIdx = index % 5;

        String title = (pos == Alliance.Position.FAR) ? FAR_TITLES[varIdx] : NEAR_TITLES[varIdx];

        return String.format(Locale.US, "%s %s: %s", pos, all, title);
    }

    private void buildAutonomousSequence(Alliance.Color alliance, Alliance.Position position, int variant, double delay) {
        AutoSequenceFactory factory = new AutoSequenceFactory(scheduler, drivetrain, intake, alliance, delay);
        if (position == Alliance.Position.FAR) {
            switch (variant) {
//                case 0: factory.buildFarCornerHusky(); break;
                case 0: factory.buildFarCornerV2(); break;
                case 1: factory.buildAleTest(); break;
                case 2: factory.bioBuzzPracticeAuto(); break;
                case 3: factory.buildBioTestTwo(); break;
//                case 2: factory.buildFarCornerOnly(); break;
//                case 3: factory.buildFarSortV3(); break;
//                case 4: factory.buildFarCornerHuskyOnly(); break;
//                default: factory.buildFarSortV3(); break;
            }
        } else {
            switch (variant) {
//                case 0: factory.buildNearGateTwice(); break;
//                case 1: factory.buildNearGateV3(); break;
//                case 2: factory.buildNearSortV2(); break;
//                case 3: factory.buildAleTest(); break;
//                case 4: factory.buildNearSortV2(); break;
//                default: factory.buildNearSortV2(); break;
            }
        }
    }

    private void updateTelemetry() {
        telemetry.addData("Scheduler", scheduler.isBusy() ? scheduler.getCurrentCommandName() : "Idle");
//        telemetry.addData("Detected Pattern", Turret.getDecodedGamePattern());
        telemetry.update();
    }
}


//
//package org.firstinspires.ftc.teamcode.opmodes;
//
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.commands.CommandBase;
//import org.firstinspires.ftc.teamcode.commands.CommandScheduler;
//import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
//import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
//import org.firstinspires.ftc.teamcode.subsystems.Intake;
//import org.firstinspires.ftc.teamcode.subsystems.Shooter;
//import org.firstinspires.ftc.teamcode.subsystems.Turret;
//
///**
// * Master Autonomous Manager.
// * Detects Alliance, Position, and Pattern automatically.
// * Allows user to select sub-variants (A, B, C, D, E) via D-pad.
// */
//@Autonomous(name = "Master Auto Manager", group = "Main")
//public class Master_Auto_Main extends OpMode {
//
//    // --- Subsystems ---
//    private RobotHardware robot;
//    private Drivetrain drivetrain;
//    private Intake intake;
//    private Shooter shooter;
//    private Turret turret;
//    private CommandScheduler scheduler;
//
//    // --- Selection State ---
//    private int selectedVariantIndex = 0; // 0=A, 1=B, 2=C, 3=D, 4=E
//    private final String[] variantLabels = {"A", "B", "C", "D", "E"};
//
//    private double startDelay = 0.0;
//
//    @Override
//    public void init() {
//        telemetry.addData("Status", "Initializing Master Manager...");
//
//        // 1. Initialize Hardware
//        robot = new RobotHardware();
//        robot.init(hardwareMap);
//
//        // 2. Initialize Subsystems
//        drivetrain = new Drivetrain(robot);
//        intake = new Intake(robot);
//        shooter = new Shooter(robot);
//        turret = new Turret(robot);
//
//        // 3. Reset Odometry
//        // We do this here because the robot is expected to be stationary on the field tiles.
//        // This recalibrates the IMU and zeros the starting position.
//        drivetrain.resetOdometry();
//
//        // 4. Initialize Scheduler
//        // Using a safe default timeout for the whole auto.
//        // Note: The CommandScheduler constructor resets the static match timer.
//        scheduler = new CommandScheduler(drivetrain, intake, shooter, turret, 29.0);
//
//        // 5. Reset Global State
//        Turret.resetGamePattern();
//
//        // 6. Start Auto Detection Mode
//        turret.startAutoDetectStart();
//        turret.setInitialPosition(0);
//
//        startDelay = 0.0; // Reset delay on init
//
//        telemetry.addData("Status", "Auto-Detecting Field State...");
//        telemetry.update();
//    }
//
//    @Override
//    public void init_loop() {
//        for (LynxModule module : robot.allHubs) {
//            module.clearBulkCache();
//        }
//
//        turret.update(drivetrain);
//
//        // Handle Variant Selection (Vertical DPAD)
//        if (gamepad1.dpadDownWasPressed()) {
//            selectedVariantIndex = (selectedVariantIndex + 1) % 5;
//        }
//        if (gamepad1.dpadUpWasPressed()) {
//            selectedVariantIndex = (selectedVariantIndex + 4) % 5;
//        }
//
//        // Handle Start Delay (Horizontal DPAD)
//        if (gamepad1.dpadRightWasPressed()) {
//            startDelay += 2.0;
//        }
//        if (gamepad1.dpadLeftWasPressed()) {
//            startDelay = Math.max(0, startDelay - 2.0); // Clamp to 0
//        }
//
//        displayDetectionUI(turret.getDetectedAlliance(), turret.getFieldPosition(),
//                         Turret.getDecodedGamePattern(), turret.getAutoDetectDist());
//    }
//
//    @Override
//    public void start() {
//        Alliance.Color alliance = turret.getDetectedAlliance();
//        if (alliance == null) {
//            alliance = Alliance.Color.BLUE; // default BLUE if not found
//            System.err.println("ERROR MASTER START: Alliance NOT detected! Defaulting to BLUE.");
//        }
//
//        Alliance.Position position = turret.getFieldPosition();
//        if (position == Alliance.Position.UNKNOWN) {
//            position = Alliance.Position.NEAR; // default NEAR if not found
//            System.err.println("ERROR MASTER START: Field Position NOT detected! Defaulting to NEAR.");
//        }
//
//        // If vision hasn't committed a pattern yet, log the error and force the PPG fallback
//        if (Turret.getDecodedGamePattern() == Turret.DecodePattern.UNKNOWN) {
//            System.err.println("ERROR MASTER START: Game Pattern NOT detected! Forcing to PPG fallback.");
//            Turret.forceSetGamePattern(Turret.DecodePattern.PPG);
//        }
//
//        System.out.printf("MASTER START: Alliance=%s, Pos=%s, Pattern=%s, Variant=%s, Delay=%.1fs%n",
//                alliance, position, Turret.getDecodedGamePattern(), variantLabels[selectedVariantIndex], startDelay);
//
//        // Build the sequence using the new Factory
//        buildAutonomousSequence(alliance, position, selectedVariantIndex, startDelay);
//
//        // Start the turret aiming to commit the pattern
//        turret.aimAtTag(alliance);
//    }
//
//    @Override
//    public void loop() {
//        for (LynxModule module : robot.allHubs) {
//            module.clearBulkCache();
//        }
//        drivetrain.update();
//        intake.update();
//        shooter.update(intake, turret, drivetrain);
//        turret.update(drivetrain);
//
//        scheduler.update();
//        updateTelemetry();
//    }
//
//    @Override
//    public void stop() {
//        CommandBase.saveAutoState(drivetrain, turret);
//
//        if (scheduler != null) {
//            scheduler.reset();
//        }
//        super.stop();
//    }
//
//    private void buildAutonomousSequence(Alliance.Color alliance, Alliance.Position position, int variant, double delay) {
//        AutoSequenceFactory factory = new AutoSequenceFactory(scheduler, drivetrain, intake, shooter, turret, alliance, delay);
//
//        if (position == Alliance.Position.FAR) {
//            switch (variant) {
//                case 0: factory.buildFarSortV3(); break;   // Variant A: Sort
//                case 1: factory.buildFarCornerV2(); break; // Variant B: Corner & Spike
//                case 2: factory.buildFarCornerOnly(); break; // Variant C: Corner only
//                case 3: factory.buildFarCornerHusky(); break;   // Variant D: Husky Test
//                case 4: factory.buildFarSortV3(); break;   // Variant E: Placeholder
//            }
//        } else {
//            switch (variant) {
//                case 0: factory.buildNearSortV2(); break; // Variant A: Sort
//                case 1: factory.buildNearGateV3(); break; // Variant B: Open Gate
//                case 2: factory.buildNearGateTwice(); break; // Variant B: Open Gate twice
//                case 3: factory.buildAleTest(); break; // Variant C: AleTest
//                case 4: factory.buildNearSortV2(); break; // Variant D: Placeholder
//            }
//        }
//    }
//
//    private void displayDetectionUI(Alliance.Color alliance, Alliance.Position position, Turret.DecodePattern pattern, double rawDist) {
//        telemetry.addLine("=== AUTO DETECTION STATUS ===");
//        telemetry.addData("Alliance", alliance == null ? "DETECTING..." : alliance);
//        telemetry.addData("Position", position == Alliance.Position.UNKNOWN ? "DETECTING..." : position);
//        telemetry.addData("Pattern", pattern == Turret.DecodePattern.UNKNOWN ? "WAITING FOR COMMIT..." : pattern);
//        telemetry.addData("Raw Tag Dist", "%.2f inches", rawDist);
//        telemetry.addData("Selected Variant", variantLabels[selectedVariantIndex]);
//        telemetry.addData("Start Delay (Left/Right)", "%.1f seconds", startDelay);
//
//        telemetry.addLine("\n=== VARIANT SELECTION (D-Pad) ===");
//        String titleA, titleB, titleC, titleD, titleE;
//        if (position == Alliance.Position.FAR) {
//            titleA = "Sort"; titleB = "Corner & Spike"; titleC = "Corner only"; titleD = "Corner Husky"; titleE = "TBD";
//        } else {
//            titleA = "Sort"; titleB = "Open gate"; titleC = "Double Gate"; titleD = "AleTest"; titleE = "TBD";
//        }
//
//        printVariantLine("A", titleA, 0);
//        printVariantLine("B", titleB, 1);
//        printVariantLine("C", titleC, 2);
//        printVariantLine("D", titleD, 3);
//        printVariantLine("E", titleE, 4);
//        telemetry.update();
//    }
//
//    private void printVariantLine(String label, String title, int index) {
//        String prefix = (selectedVariantIndex == index) ? " > [*] " : "    [  ] ";
//        telemetry.addLine(prefix + label + ": " + title);
//    }
//
//    private void updateTelemetry() {
//        telemetry.addData("Scheduler", scheduler.isBusy() ? "Running: " + scheduler.getCurrentCommandName() : "Idle");
//        telemetry.addData("Detected Pattern", Turret.getDecodedGamePattern());
//        telemetry.addData("Turret Mode", turret.getMode());
//        telemetry.update();
//    }
//}
