package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.commands.*;
import org.firstinspires.ftc.teamcode.pathing.CurvePoint;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.util.Alliance;

import java.util.ArrayList;

/**
 * Factory class to build autonomous sequences for any alliance and position.
 * Handles mirroring coordinates and angles automatically.
 */
public class AutoSequenceFactory {

    private final CommandScheduler scheduler;
    private final Drivetrain drivetrain;
    private final Intake intake;
//    private final Shooter shooter;
//    private final Turret turret;
    private final Alliance.Color alliance;
    private final boolean isRed;
    private final double startDelay;

    public AutoSequenceFactory(CommandScheduler scheduler, Drivetrain drivetrain, Intake intake, Alliance.Color alliance, double startDelay) {
        this.scheduler = scheduler;
        this.drivetrain = drivetrain;
        this.intake = intake;
//        this.shooter = shooter;
//        this.turret = turret;
        this.alliance = alliance;
        this.isRed = (alliance == Alliance.Color.RED);
        this.startDelay = startDelay;
    }

    /**
     * Helper to mirror Y coordinates based on alliance.
     * Blue is positive Y, Red is negative Y.
     */
    private double y(double val) {
        return isRed ? -val : val;
    }

    /**
     * Mirrored heading in radians for Near Sort/Gate routines.
     * Often 0 becomes 180 and 180 becomes 0 when switching sides.
     */
    private double mirroredHeading(double degrees) {
        if (!isRed) return Math.toRadians(degrees);
        
        // Specific mirroring logic for headings that need to flip 180 degrees
        if (degrees == 0) return Math.toRadians(180);
        if (degrees == 180) return Math.toRadians(0);
        
        return Math.toRadians(degrees);
    }

    /**
     * FAR Variant B: Corner & Spike
     */
    public void buildFarCornerV2() {
        commonFarInit();

        // 1. Preload
//        scheduler.add(new SetShooterPreloadModeCommand().withName("set to preload"));

        // 2. Drive to Shooting Spot
        ArrayList<CurvePoint> pathToShoot = new ArrayList<>();
        boolean debug = false;
        pathToShoot.add(new CurvePoint(-62.04, y(14.00), 1.00, 1.00, 5, Math.toRadians(60.0), 1.00));
        pathToShoot.add(new CurvePoint(-54.90, y(11.88), 1.00, 1.00, 5, Math.toRadians(60.0), 1.00));
        pathToShoot.add(new CurvePoint(-49.00, y(11.88), 0.80, 1.00, 5, Math.toRadians(60.0), 1.00));
        scheduler.add(new FollowPathCommand(pathToShoot, Math.toRadians(90), debug)
                .transitionImmediately()
                .withName("Drive to Shooting Spot"));

        scheduler.add(new WaitCommand(1.5));
//        scheduler.add(new Shoot3BallsCommand().withName("Shoot first 3 Balls"));

        // 3. Drive to Corner
        ArrayList<CurvePoint> driveToCornerA = new ArrayList<>();
        driveToCornerA.add(new CurvePoint(-50.70, y(11.70), 1.00, 0.40, 10.00, Math.toRadians(60.0), 0.60));
        driveToCornerA.add(new CurvePoint(-50.70, y(50.22), 1.00, 0.40, 5.00, Math.toRadians(60.0), 0.60));
        driveToCornerA.add(new CurvePoint(-50.70, y(63.50), 1.00, 0.40, 5.00, Math.toRadians(60.0), 0.60));
        double headingA = isRed ? 75 : 115;
        scheduler.add(new FollowPathCommand(driveToCornerA, Math.toRadians(headingA), debug, 4)
                .withName("Drive to Corner A"));

        scheduler.add(new WaitCommand(.3));

        ArrayList<CurvePoint> driveToCornerB = new ArrayList<>();
        driveToCornerB.add(new CurvePoint(-50.58, y(63.02), 1.00, 0.40, 10.00, Math.toRadians(60.0), 0.60));
        driveToCornerB.add(new CurvePoint(-55.26, y(47.76), 1.00, 0.40, 10.00, Math.toRadians(60.0), 0.60));
        scheduler.add(new FollowPathCommand(driveToCornerB, Math.toRadians(-90), debug, 2)
                .withName("Drive to Corner B"));

        ArrayList<CurvePoint> driveToCornerC = new ArrayList<>();
        driveToCornerC.add(new CurvePoint(-55.26, y(47.76), 1.00, 0.40, 10.00, Math.toRadians(60.0), 0.60));
        driveToCornerC.add(new CurvePoint(-61.02, y(59.76), 1.00, 0.40, 10.00, Math.toRadians(60.0), 0.60));
        scheduler.add(new FollowPathCommand(driveToCornerC, Math.toRadians(90), debug, 2)
                .withName("Drive to Corner C"));

        scheduler.add(new WaitCommand(0.3));

//        ArrayList<CurvePoint> driveToCornerD = new ArrayList<>();
//        driveToCornerD.add(new CurvePoint(-61.02, y(59.76), 1.00, 0.40, 10.00, Math.toRadians(60.0), 0.60));
//        driveToCornerD.add(new CurvePoint(-59.02, y(47.14), 1.00, 0.40, 10.00, Math.toRadians(60.0), 0.60));
//        scheduler.add(new FollowPathCommand(driveToCornerD, Math.toRadians(-90), debug, 2)
//                .withName("Drive to Corner D"));
//
//        scheduler.add(new WaitForDrivetrainCommand());
//
//        ArrayList<CurvePoint> driveToCornerE = new ArrayList<>();
//        driveToCornerE.add(new CurvePoint(-59.02, y(47.14), 1.00, 0.40, 10.00, Math.toRadians(60.0), 0.60));
//        driveToCornerE.add(new CurvePoint(-61.02, y(59.76), 1.00, 0.40, 10.00, Math.toRadians(60.0), 0.60));
//        scheduler.add(new FollowPathCommand(driveToCornerE, Math.toRadians(90), debug, 2)
//                .withName("Drive to Corner E"));
//
//        scheduler.add(new WaitCommand(0.3));

        // 4. Drive to Shoot Spot 2
        ArrayList<CurvePoint> driveToShoot2 = new ArrayList<>();
        driveToShoot2.add(new CurvePoint(-61.20, y(58.86), 0.40, 0.40, 10.00, Math.toRadians(60.0), 0.60));
        driveToShoot2.add(new CurvePoint(-53.64, y(15.48), 1.00, 1.00, 10.00, Math.toRadians(60.0), 1.00));
        double headingShoot2 = isRed ? 0 : 180;
        scheduler.add(new FollowPathCommand(driveToShoot2, Math.toRadians(headingShoot2), debug)
                .withName("Drive to Shoot spot 2"));

//        scheduler.add(new Shoot3BallsCommand().withName("Shoot second 3 Balls"));

        // 5. Drive to Spike 1
        ArrayList<CurvePoint> pathToSpike1 = new ArrayList<>();
        pathToSpike1.add(new CurvePoint(-49.14, y(11.88), 1.00, 1.00, 10.00, Math.toRadians(60.0), 1.00));
        pathToSpike1.add(new CurvePoint(-36.00, y(22.14), 1.00, 1.00, 10.00, Math.toRadians(60.0), 1.00));
        pathToSpike1.add(new CurvePoint(-35.46, y(42.66), 0.80, 0.80, 10.00, Math.toRadians(60.0), 0.60));
        pathToSpike1.add(new CurvePoint(-35.28, y(51.82), 0.40, 0.80, 10.00, Math.toRadians(60.0), 0.60));
        scheduler.add(new FollowPathCommand(pathToSpike1, Math.toRadians(90), debug).withName("Drive to Spike 1"));

        // 6. Drive to Shoot Spot 3
        ArrayList<CurvePoint> pathToShot3 = new ArrayList<>();
        pathToShot3.add(new CurvePoint(-35.28, y(51.84), 1.00, 1.00, 5.00, Math.toRadians(60.0), 1.00));
        pathToShot3.add(new CurvePoint(-46.62, y(12.62), 1.00, 1.00, 5.00, Math.toRadians(60.0), 1.00));
        scheduler.add(new FollowPathCommand(pathToShot3, Math.toRadians(-90), debug)
                .withName("Drive to Shoot Spot 3"));

//        scheduler.add(new Shoot3BallsCommand().withName("Shoot third 3 Balls"));


        ArrayList<CurvePoint> driveToCornerH = new ArrayList<>();
        driveToCornerH.add(new CurvePoint(-50.70, y(11.70), 1.00, 0.40, 10.00, Math.toRadians(60.0), 0.60));
        driveToCornerH.add(new CurvePoint(-54.54, y(50.04), 1.00, 0.40, 5.00, Math.toRadians(60.0), 0.60));
        driveToCornerH.add(new CurvePoint(-58.68, y(62.46), 1.00, 0.40, 5.00, Math.toRadians(60.0), 0.60));
        headingA = isRed ? 75 : 115;
        scheduler.add(new FollowPathCommand(driveToCornerH, Math.toRadians(headingA), debug, 4)
                .withName("Drive In Bump 3"));

        scheduler.add(new WaitCommand(.3));

        ArrayList<CurvePoint> driveToCornerI = new ArrayList<>();
        driveToCornerI.add(new CurvePoint(-58.68, y(62.46), 1.00, 0.40, 10.00, Math.toRadians(60.0), 0.60));
        driveToCornerI.add(new CurvePoint(-55.26, y(47.76), 1.00, 0.40, 10.00, Math.toRadians(60.0), 0.60));
        scheduler.add(new FollowPathCommand(driveToCornerI, Math.toRadians(-90), debug, 2)
                .withName("Drive Out Bump 3"));

        ArrayList<CurvePoint> driveToShoot4 = new ArrayList<>();
        driveToShoot4.add(new CurvePoint(-61.20, y(58.86), 0.40, 0.40, 10.00, Math.toRadians(60.0), 0.60));
        driveToShoot4.add(new CurvePoint(-53.64, y(15.48), 1.00, 1.00, 10.00, Math.toRadians(60.0), 1.00));
        double headingShoot4 = isRed ? 0 : 180;
        scheduler.add(new FollowPathCommand(driveToShoot4, Math.toRadians(headingShoot4), debug)
                .withName("Drive to Shoot spot 4"));

//        scheduler.add(new Shoot3BallsCommand().withName("Shoot fourth 3 Balls"));

        commonFarTeardown();
    }

    /**
     * FAR Variant C: Corner Only
     */
//    public void buildFarCornerOnly() {
//        commonFarInit();
//
//        // 1. Preload
////        scheduler.add(new SetShooterPreloadModeCommand().withName("set to preload"));
//
//        // 2. Drive to Shooting Spot
//        ArrayList<CurvePoint> pathToShoot = new ArrayList<>();
//        boolean debug = false;
//        pathToShoot.add(new CurvePoint(-62.04, y(14.00), 1.00, 1.00, 5, Math.toRadians(60.0), 1.00));
//        pathToShoot.add(new CurvePoint(-54.90, y(11.88), 1.00, 1.00, 5, Math.toRadians(60.0), 1.00));
//        pathToShoot.add(new CurvePoint(-49.00, y(11.88), 0.80, 1.00, 5, Math.toRadians(60.0), 1.00));
//        scheduler.add(new FollowPathCommand(pathToShoot, Math.toRadians(90), debug)
//                .transitionImmediately()
//                .withName("Drive to Shooting Spot"));
//
//        scheduler.add(new WaitCommand(1.5));
////        scheduler.add(new Shoot3BallsCommand().withName("Shoot first 3 Balls"));
//
//        // 3. Drive to Corner
//        ArrayList<CurvePoint> driveToCornerA = new ArrayList<>();
//        driveToCornerA.add(new CurvePoint(-50.70, y(11.70), 1.00, 0.40, 10.00, Math.toRadians(60.0), 0.60));
//        driveToCornerA.add(new CurvePoint(-54.54, y(50.04), 1.00, 0.40, 5.00, Math.toRadians(60.0), 0.60));
//        driveToCornerA.add(new CurvePoint(-58.68, y(62.46), 1.00, 0.40, 5.00, Math.toRadians(60.0), 0.60));
//        double headingA = isRed ? 75 : 115;
//        scheduler.add(new FollowPathCommand(driveToCornerA, Math.toRadians(headingA), debug, 2.5)
//                .withName("Drive In Bump 1"));
//
//        scheduler.add(new WaitCommand(.3));
//
//        ArrayList<CurvePoint> driveToCornerB = new ArrayList<>();
//        driveToCornerB.add(new CurvePoint(-58.68, y(62.46), 1.00, 0.40, 10.00, Math.toRadians(60.0), 0.60));
//        driveToCornerB.add(new CurvePoint(-55.26, y(47.76), 1.00, 0.40, 10.00, Math.toRadians(60.0), 0.60));
//        scheduler.add(new FollowPathCommand(driveToCornerB, Math.toRadians(-90), debug, 2)
//                .withName("Drive Out Bump 1"));
//
//        ArrayList<CurvePoint> driveToCornerC = new ArrayList<>();
//        driveToCornerC.add(new CurvePoint(-55.26, y(47.76), 1.00, 0.40, 10.00, Math.toRadians(60.0), 0.60));
//        driveToCornerC.add(new CurvePoint(-61.02, y(59.76), 1.00, 0.40, 10.00, Math.toRadians(60.0), 0.60));
//        scheduler.add(new FollowPathCommand(driveToCornerC, Math.toRadians(90), debug, 2)
//                .withName("Drive In Bump 2"));
//
//        scheduler.add(new WaitCommand(0.3));
////
////        ArrayList<CurvePoint> driveToCornerD = new ArrayList<>();
////        driveToCornerD.add(new CurvePoint(-61.02, y(59.76), 1.00, 0.40, 10.00, Math.toRadians(60.0), 0.60));
////        driveToCornerD.add(new CurvePoint(-59.02, y(47.14), 1.00, 0.40, 10.00, Math.toRadians(60.0), 0.60));
////        scheduler.add(new FollowPathCommand(driveToCornerD, Math.toRadians(-90), debug, 2)
////                .withName("Drive to Corner D"));
//
////        scheduler.add(new WaitForDrivetrainCommand());
//
////        ArrayList<CurvePoint> driveToCornerE = new ArrayList<>();
////        driveToCornerE.add(new CurvePoint(-59.02, y(47.14), 1.00, 0.40, 10.00, Math.toRadians(60.0), 0.60));
////        driveToCornerE.add(new CurvePoint(-61.02, y(59.76), 1.00, 0.40, 10.00, Math.toRadians(60.0), 0.60));
////        scheduler.add(new FollowPathCommand(driveToCornerE, Math.toRadians(90), debug, 2)
////                .withName("Drive to Corner E"));
//
////        scheduler.add(new WaitCommand(0.3));
//
//        // 4. Drive to Shoot Spot 2
//        ArrayList<CurvePoint> driveToShoot2 = new ArrayList<>();
//        driveToShoot2.add(new CurvePoint(-61.20, y(58.86), 0.40, 0.40, 10.00, Math.toRadians(60.0), 0.60));
//        driveToShoot2.add(new CurvePoint(-53.64, y(15.48), 1.00, 1.00, 10.00, Math.toRadians(60.0), 1.00));
//        double headingShoot2 = isRed ? 0 : 180;
//        scheduler.add(new FollowPathCommand(driveToShoot2, Math.toRadians(headingShoot2), debug)
//                .withName("Drive to Shoot spot 2"));
//
//        scheduler.add(new Shoot3BallsCommand().withName("Shoot second 3 Balls"));
//
//        ArrayList<CurvePoint> driveToCornerF = new ArrayList<>();
//        driveToCornerF.add(new CurvePoint(-50.70, y(11.70), 1.00, 0.40, 10.00, Math.toRadians(60.0), 0.60));
//        driveToCornerF.add(new CurvePoint(-54.54, y(50.04), 1.00, 0.40, 5.00, Math.toRadians(60.0), 0.60));
//        driveToCornerF.add(new CurvePoint(-58.68, y(62.46), 1.00, 0.40, 5.00, Math.toRadians(60.0), 0.60));
//        headingA = isRed ? 75 : 115;
//        scheduler.add(new FollowPathCommand(driveToCornerF, Math.toRadians(headingA), debug, 2)
//                .withName("Drive In Bump 3"));
//
////        scheduler.add(new WaitCommand(.3));
//
//        ArrayList<CurvePoint> driveToCornerG = new ArrayList<>();
//        driveToCornerG.add(new CurvePoint(-58.68, y(62.46), 1.00, 0.40, 10.00, Math.toRadians(60.0), 0.60));
//        driveToCornerG.add(new CurvePoint(-55.26, y(47.76), 1.00, 0.40, 10.00, Math.toRadians(60.0), 0.60));
//        scheduler.add(new FollowPathCommand(driveToCornerG, Math.toRadians(-90), debug, 2)
//                .withName("Drive Out Bump 3"));
//
//        ArrayList<CurvePoint> driveToShoot3 = new ArrayList<>();
//        driveToShoot3.add(new CurvePoint(-61.20, y(58.86), 0.40, 0.40, 10.00, Math.toRadians(60.0), 0.60));
//        driveToShoot3.add(new CurvePoint(-53.64, y(15.48), 1.00, 1.00, 10.00, Math.toRadians(60.0), 1.00));
//        double headingShoot3 = isRed ? 0 : 180;
//        scheduler.add(new FollowPathCommand(driveToShoot2, Math.toRadians(headingShoot3), debug)
//                .withName("Drive to Shoot spot 3"));
//
//        scheduler.add(new Shoot3BallsCommand().withName("Shoot third 3 Balls"));
//
//        ArrayList<CurvePoint> driveToCornerH = new ArrayList<>();
//        driveToCornerH.add(new CurvePoint(-50.70, y(11.70), 1.00, 0.40, 10.00, Math.toRadians(60.0), 0.60));
//        driveToCornerH.add(new CurvePoint(-54.54, y(50.04), 1.00, 0.40, 5.00, Math.toRadians(60.0), 0.60));
//        driveToCornerH.add(new CurvePoint(-58.68, y(62.46), 1.00, 0.40, 5.00, Math.toRadians(60.0), 0.60));
//        headingA = isRed ? 75 : 115;
//        scheduler.add(new FollowPathCommand(driveToCornerH, Math.toRadians(headingA), debug, 2)
//                .withName("Drive In Bump 4"));
//
//        scheduler.add(new WaitCommand(.3));
//
//        ArrayList<CurvePoint> driveToCornerI = new ArrayList<>();
//        driveToCornerI.add(new CurvePoint(-58.68, y(62.46), 1.00, 0.40, 10.00, Math.toRadians(60.0), 0.60));
//        driveToCornerI.add(new CurvePoint(-55.26, y(47.76), 1.00, 0.40, 10.00, Math.toRadians(60.0), 0.60));
//        scheduler.add(new FollowPathCommand(driveToCornerI, Math.toRadians(-90), debug, 2)
//                .withName("Drive Out Bump 4"));
//
//
//        ArrayList<CurvePoint> driveToShoot4 = new ArrayList<>();
//        driveToShoot4.add(new CurvePoint(-61.20, y(58.86), 0.40, 0.40, 10.00, Math.toRadians(60.0), 0.60));
//        driveToShoot4.add(new CurvePoint(-53.64, y(15.48), 1.00, 1.00, 10.00, Math.toRadians(60.0), 1.00));
//        double headingShoot4 = isRed ? 0 : 180;
//        scheduler.add(new FollowPathCommand(driveToShoot4, Math.toRadians(headingShoot4), debug)
//                .withName("Drive to Shoot spot 4"));
//
//        scheduler.add(new Shoot3BallsCommand().withName("Shoot fourth 3 Balls"));
//
//        commonFarTeardown();
//    }
//
//    /**
//     * FAR Variant A: Sort
//     */
//    public void buildFarSortV3() {
//        commonFarInit();
//
//        scheduler.add(new CloseAllGatesCommand().withName("Close gates before moving"));
//
//        ArrayList<CurvePoint> pathToShoot = new ArrayList<>();
//        boolean debug = false;
//        pathToShoot.add(new CurvePoint(-62.04, y(14.00), 1.00, 1.00, 5, Math.toRadians(60.0), 1.00));
//        pathToShoot.add(new CurvePoint(-54.90, y(11.88), 1.00, 1.00, 5, Math.toRadians(60.0), 1.00));
//        pathToShoot.add(new CurvePoint(-46.98, y(11.88), 0.60, 1.00, 5, Math.toRadians(60.0), 1.00));
//        scheduler.add(new FollowPathCommand(pathToShoot, Math.toRadians(90), debug)
//                .transitionImmediately()
//                .withName("Drive to Shooting Spot"));
//
//        scheduler.add(new ShootByPatternCommand().withName("Shoot first 3 Balls by pattern"));
//        scheduler.add(new SetShooterPreloadModeCommand().withName("set to preload"));
//
//        ArrayList<CurvePoint> pathToSpike1 = new ArrayList<>();
//        pathToSpike1.add(new CurvePoint(-49.14, y(11.88), 1.00, 1.00, 5.00, Math.toRadians(60.0), 1.00));
//        pathToSpike1.add(new CurvePoint(-38.30, y(21.24), 0.90, 1.00, 5.00, Math.toRadians(60.0), 1.00));
//        pathToSpike1.add(new CurvePoint(-35.46, y(42.66), 0.50, 0.60, 5.00, Math.toRadians(60.0), 0.60));
//        pathToSpike1.add(new CurvePoint(-34.00, y(53.28), 0.30, 0.40, 5.00, Math.toRadians(60.0), 0.60));
//        scheduler.add(new FollowPathCommand(pathToSpike1, Math.toRadians(90), debug).withName("Drive to Spike 1"));
//
//        ArrayList<CurvePoint> pathToShot3 = new ArrayList<>();
//        pathToShot3.add(new CurvePoint(-35.28, y(51.84), 1.00, 1.00, 5, Math.toRadians(60.0), 1.00));
//        pathToShot3.add(new CurvePoint(-35.46, y(42.66), 1.00, 1.00, 5, Math.toRadians(60.0), 1.00));
//        pathToShot3.add(new CurvePoint(-35.30, y(23.76), 1.00, 1.00, 5, Math.toRadians(60.0), 1.00));
//        pathToShot3.add(new CurvePoint(-47.00, y(11.20), 1.00, 1.00, 5, Math.toRadians(60.0), 1.00));
//        scheduler.add(new FollowPathCommand(pathToShot3, Math.toRadians(-90), debug)
//                .withName("Drive to Shoot Spot 3"));
//
//        scheduler.add(new Shoot3BallsCommand().withName("Shoot third 3 Balls"));
//
//        commonFarTeardown();
//    }
//
//    /**
//     * NEAR Variant A: Sort
//     */
//    public void buildNearSortV2() {
//        commonNearInit();
//
//        scheduler.add(new CloseAllGatesCommand().withName("Close gates before moving"));
//        scheduler.add(new SeekAndAimCommand(0, alliance).transitionImmediately().withName("Aim at 0 degrees"));
//
//        ArrayList<CurvePoint> pathToShoot = new ArrayList<>();
//        boolean debug = false;
//        pathToShoot.add(new CurvePoint(41.04, y(54.54), 0.75, 0.85, 10.00, Math.toRadians(60.0), 0.90));
//        pathToShoot.add(new CurvePoint(40.68, y(46.44), 1.00, 0.95, 10.00, Math.toRadians(60.0), 0.90));
//        pathToShoot.add(new CurvePoint(39.96, y(39.42), 1.00, 0.95, 10.00, Math.toRadians(60.0), 0.90));
//        pathToShoot.add(new CurvePoint(30.60, y(28.98), 1.00, 0.90, 10.00, Math.toRadians(60.0), 0.90));
//        scheduler.add(new FollowPathCommand(pathToShoot, Math.toRadians(-90), debug)
//                .transitionImmediately()
//                .withName("Drive to Shooting Spot 1"));
//
//        scheduler.add(new ShootByPatternCommand().withName("Shoot first 3 Balls by pattern"));
//        scheduler.add(new SetShooterPreloadModeCommand().withName("set to preload"));
//
//        ArrayList<CurvePoint> pathToSpike1 = new ArrayList<>();
//        pathToSpike1.add(new CurvePoint(30.60, y(28.98), 0.70, 0.85, 7.00, Math.toRadians(60.0), 0.60));
//        pathToSpike1.add(new CurvePoint(17.28, y(16.56), 0.70, 1.00, 7.00, Math.toRadians(60.0), 0.90));
//        pathToSpike1.add(new CurvePoint(11.88, y(16.74), 0.70, 1.00, 5.00, Math.toRadians(60.0), 0.90));
//        pathToSpike1.add(new CurvePoint(11.88, y(30.42), 0.90, 1.00, 7.00, Math.toRadians(60.0), 0.90));
//        pathToSpike1.add(new CurvePoint(11.88, y(38.70), 0.70, 0.85, 7.00, Math.toRadians(60.0), 0.90));
//        pathToSpike1.add(new CurvePoint(11.88, y(52.02), 0.40, 0.85, 2.00, Math.toRadians(60.0), 0.90));
//        scheduler.add(new FollowPathCommand(pathToSpike1, Math.toRadians(90), debug).withName("Drive to Spike 1"));
//
//        ArrayList<CurvePoint> pathToShot2 = new ArrayList<>();
//        pathToShot2.add(new CurvePoint(12.24, y(48.06), 1.00, 0.40, 10.00, Math.toRadians(60.0), 0.60));
//        pathToShot2.add(new CurvePoint(32.76, y(31.86), 1.00, 1.00, 10.00, Math.toRadians(60.0), 1.00));
//        scheduler.add(new FollowPathCommand(pathToShot2, mirroredHeading(180), debug)
//                .withName("Drive to Shoot Spot 2"));
//
//        scheduler.add(new SetShooterPreloadModeCommand().withName("set to preload"));
//        scheduler.add(new Shoot3BallsCommand().withName("Shoot second 3 Balls"));
//
//        ArrayList<CurvePoint> pathToSpike2a = new ArrayList<>();
//        pathToSpike2a.add(new CurvePoint(33.66, y(31.50), 0.40, 0.40, 10.00, Math.toRadians(60.0), 0.60));
//        pathToSpike2a.add(new CurvePoint(-11.34, y(23.22), 1.00, 1.00, 10.00, Math.toRadians(60.0), 1.00));
//        scheduler.add(new FollowPathCommand(pathToSpike2a, mirroredHeading(0), debug, 8).withName("Drive to Spike 2a"));
//
//        ArrayList<CurvePoint> pathToSpike2b = new ArrayList<>();
//        pathToSpike2b.add(new CurvePoint(-11.52, y(24.12), 0.40, 0.40, 10.00, Math.toRadians(60.0), 0.60));
//        pathToSpike2b.add(new CurvePoint(-11.70, y(34.38), 0.90, 1.00, 10.00, Math.toRadians(60.0), 1.00));
//        pathToSpike2b.add(new CurvePoint(-11.70, y(46.80), 0.40, 0.90, 8.00, Math.toRadians(60.0), 0.90));
//        scheduler.add(new FollowPathCommand(pathToSpike2b, Math.toRadians(90), debug, 8).withName("Drive to Spike 2b"));
//
//        ArrayList<CurvePoint> pathToShot3 = new ArrayList<>();
//        pathToShot3.add(new CurvePoint(-11.88, y(46.62), 1.00, 1.00, 10.00, Math.toRadians(60.0), 0.90));
//        pathToShot3.add(new CurvePoint(43.02, y(21.78), 1.00, 1.00, 10.00, Math.toRadians(60.0), 1.00));
//        scheduler.add(new FollowPathCommand(pathToShot3, mirroredHeading(180), debug, 5)
//                .transitionWhenDistancetoEndIsLessThan(20)
//                .withName("Drive to Shot 3"));
//
//        scheduler.add(new Shoot3BallsCommand().withName("Shoot third 3 Balls"));
//        scheduler.add(new EndCommand().withName("End of auto"));
//    }
//
//    /**
//     * NEAR Variant B: Open Gate (no sort)
//     */
//    public void buildNearGateV3() {
//        commonNearInit();
//
//        scheduler.add(new SetShooterPreloadModeCommand().withName("set to preload"));
//        scheduler.add(new SeekAndAimCommand(0, alliance).transitionImmediately().withName("Aim at 0 degrees"));
//
//        ArrayList<CurvePoint> pathToShoot = new ArrayList<>();
//        boolean debug = false;
//        pathToShoot.add(new CurvePoint(41.04, y(54.54), 0.75, 0.85, 10.00, Math.toRadians(60.0), 0.90));
//        pathToShoot.add(new CurvePoint(40.68, y(46.44), 1.00, 0.95, 10.00, Math.toRadians(60.0), 0.90));
//        pathToShoot.add(new CurvePoint(39.96, y(39.42), 1.00, 0.95, 10.00, Math.toRadians(60.0), 0.90));
//        pathToShoot.add(new CurvePoint(24.30, y(23.94), 1.00, 0.90, 10.00, Math.toRadians(60.0), 0.90));
//        scheduler.add(new FollowPathCommand(pathToShoot, Math.toRadians(-90), debug)
//                .withName("Drive to Shooting Spot 1"));
//
//        scheduler.add(new Shoot3BallsCommand().withName("Shoot first 3 Balls"));
//
//        ArrayList<CurvePoint> pathToSpike2a = new ArrayList<>();
//        pathToSpike2a.add(new CurvePoint(24.30, y(23.94), 0.40, 0.40, 10.00, Math.toRadians(60.0), 0.60));
//        pathToSpike2a.add(new CurvePoint(-11.34, y(23.22), 1.00, 1.00, 10.00, Math.toRadians(60.0), 1.00));
//        scheduler.add(new FollowPathCommand(pathToSpike2a, mirroredHeading(0), debug, 8).withName("Drive to Spike 2a"));
//
//        ArrayList<CurvePoint> pathToSpike2b = new ArrayList<>();
//        pathToSpike2b.add(new CurvePoint(-11.52, y(24.12), 0.40, 0.40, 10.00, Math.toRadians(60.0), 0.60));
//        pathToSpike2b.add(new CurvePoint(-11.70, y(34.38), 0.90, 1.00, 10.00, Math.toRadians(60.0), 1.00));
//        pathToSpike2b.add(new CurvePoint(-11.70, y(47.70), 0.55, 0.95, 9.00, Math.toRadians(60.0), 0.95));
//        pathToSpike2b.add(new CurvePoint(-5.04, y(54.54), 0.40, 0.90, 8.00, Math.toRadians(60.0), 0.90));
//        scheduler.add(new FollowPathCommand(pathToSpike2b, Math.toRadians(90), debug, 8).withName("Drive to Spike 2b"));
//
//        ArrayList<CurvePoint> pathToShot2 = new ArrayList<>();
//        pathToShot2.add(new CurvePoint(-5.04, y(54.54), 1.00, 1.00, 10.00, Math.toRadians(60.0), 0.60));
//        pathToShot2.add(new CurvePoint(-0.18, y(26.46), 1.00, 1.00, 10.00, Math.toRadians(60.0), 0.70));
//        pathToShot2.add(new CurvePoint(12.06, y(11.88), 1.00, 1.00, 10.00, Math.toRadians(60.0), 1.00));
//        scheduler.add(new FollowPathCommand(pathToShot2, mirroredHeading(180), debug).withName("Drive to Shoot Spot 2"));
//
//        scheduler.add(new Shoot3BallsCommand().withName("Shoot Second 3 Balls"));
//
//        ArrayList<CurvePoint> pathToSpike1A = new ArrayList<>();
//        pathToSpike1A.add(new CurvePoint(12.06, y(11.88), 1.00, 1.00, 5.00, Math.toRadians(60.0), 0.90));
//        pathToSpike1A.add(new CurvePoint(11.88, y(30.42), 1.00, 1.00, 7.00, Math.toRadians(60.0), 0.90));
//        pathToSpike1A.add(new CurvePoint(11.88, y(38.70), 0.80, 0.85, 7.00, Math.toRadians(60.0), 0.90));
//        pathToSpike1A.add(new CurvePoint(11.88, y(52.02), 0.55, 0.85, 2.00, Math.toRadians(60.0), 0.90));
//        scheduler.add(new FollowPathCommand(pathToSpike1A, Math.toRadians(90), debug).withName("Drive to Spike 1A"));
//
//        ArrayList<CurvePoint> pathToShot3 = new ArrayList<>();
//        pathToShot3.add(new CurvePoint(11.88, y(52.02), 1.00, 1.00, 10.00, Math.toRadians(60.0), 0.90));
//        pathToShot3.add(new CurvePoint(43.02, y(21.78), 1.00, 1.00, 10.00, Math.toRadians(60.0), 1.00));
//        scheduler.add(new FollowPathCommand(pathToShot3, mirroredHeading(180), debug, 5)
//                .transitionWhenDistancetoEndIsLessThan(20)
//                .withName("Drive to Shot 3"));
//
//        scheduler.add(new Shoot3BallsCommand().withName("Shoot third 3 Balls"));
//        scheduler.add(new EndCommand().withName("End of auto"));
//    }
//
//    /**
//     * NEAR Variant B: Open Gate (no sort)
//     */
//    public void buildNearGateTwice() {
//        commonNearInit();
//
//        scheduler.add(new SetShooterPreloadModeCommand().withName("set to preload"));
//        scheduler.add(new SeekAndAimCommand(0, alliance).transitionImmediately().withName("Aim at 0 degrees"));
//
//        ArrayList<CurvePoint> pathToShoot = new ArrayList<>();
//        boolean debug = false;
//        pathToShoot.add(new CurvePoint(41.04, y(54.54), 0.75, 0.85, 10.00, Math.toRadians(60.0), 0.90));
//        pathToShoot.add(new CurvePoint(40.68, y(46.44), 1.00, 0.95, 10.00, Math.toRadians(60.0), 0.90));
//        pathToShoot.add(new CurvePoint(39.96, y(39.42), 1.00, 0.95, 10.00, Math.toRadians(60.0), 0.90));
//        pathToShoot.add(new CurvePoint(24.30, y(23.94), 1.00, 0.90, 10.00, Math.toRadians(60.0), 0.90));
//        scheduler.add(new FollowPathCommand(pathToShoot, Math.toRadians(-90), debug)
//                .withName("Drive to Shooting Spot 1"));
//
//        scheduler.add(new Shoot3BallsCommand().withName("Shoot first 3 Balls"));
//
//        ArrayList<CurvePoint> pathToSpike2a = new ArrayList<>();
//        pathToSpike2a.add(new CurvePoint(24.30, y(23.94), 0.40, 0.40, 10.00, Math.toRadians(60.0), 0.60));
//        pathToSpike2a.add(new CurvePoint(-11.34, y(23.22), 1.00, 1.00, 10.00, Math.toRadians(60.0), 1.00));
//        scheduler.add(new FollowPathCommand(pathToSpike2a, mirroredHeading(0), debug, 8).withName("Drive to Spike 2a"));
//
//        ArrayList<CurvePoint> pathToSpike2b = new ArrayList<>();
//        pathToSpike2b.add(new CurvePoint(-11.52, y(24.12), 0.40, 0.40, 10.00, Math.toRadians(60.0), 0.60));
//        pathToSpike2b.add(new CurvePoint(-11.70, y(34.38), 0.90, 1.00, 10.00, Math.toRadians(60.0), 1.00));
//        pathToSpike2b.add(new CurvePoint(-11.70, y(47.70), 0.55, 0.95, 9.00, Math.toRadians(60.0), 0.95));
//        pathToSpike2b.add(new CurvePoint(-1.08, y(56.88), 0.40, 0.90, 8.00, Math.toRadians(60.0), 0.90));
//        scheduler.add(new FollowPathCommand(pathToSpike2b, Math.toRadians(90), debug, 2).withName("Drive to Spike 2b"));
//
//        ArrayList<CurvePoint> pathToShot2 = new ArrayList<>();
//        pathToShot2.add(new CurvePoint(-5.04, y(54.54), 1.00, 1.00, 10.00, Math.toRadians(60.0), 0.60));
//        pathToShot2.add(new CurvePoint(-2.52, y(24.48), 1.00, 1.00, 10.00, Math.toRadians(60.0), 0.70));
//        pathToShot2.add(new CurvePoint(3.78, y(3.24), 1.00, 1.00, 10.00, Math.toRadians(60.0), 1.00));
//        scheduler.add(new FollowPathCommand(pathToShot2, mirroredHeading(180), debug).withName("Drive to Shoot Spot 2"));
//
//        scheduler.add(new Shoot3BallsCommand().withName("Shoot Second 3 Balls"));
//
//        ArrayList<CurvePoint> pathToGate2 = new ArrayList<>();
//        pathToGate2.add(new CurvePoint(12.06, y(11.88), 0.40, 0.40, 10.00, Math.toRadians(60.0), 0.60));
//        pathToGate2.add(new CurvePoint(-11.70, y(34.38), 1, 1.00, 10.00, Math.toRadians(60.0), 1.00));
//        pathToGate2.add(new CurvePoint(-16.02, y(45.54), 0.8, 0.95, 9.00, Math.toRadians(60.0), 0.95));
//        pathToGate2.add(new CurvePoint(-9.00, y(59.04), 0.7, 0.90, 8.00, Math.toRadians(60.0), 0.90));
//        scheduler.add(new FollowPathCommand(pathToGate2, Math.toRadians(90), debug, 3.5).withName("Drive to Open Gate 2"));
//
//        scheduler.add(new WaitCommand(1.5));
//
//        ArrayList<CurvePoint> pathToShot3 = new ArrayList<>();
//        pathToShot3.add(new CurvePoint(-5.04, y(54.54), 1.00, 1.00, 10.00, Math.toRadians(60.0), 0.60));
//        pathToShot3.add(new CurvePoint(-0.18, y(26.46), 1.00, 1.00, 10.00, Math.toRadians(60.0), 0.70));
//        pathToShot3.add(new CurvePoint(12.06, y(11.88), 1.00, 1.00, 10.00, Math.toRadians(60.0), 1.00));
//        scheduler.add(new FollowPathCommand(pathToShot3, mirroredHeading(180), debug).withName("Drive to Shoot Spot 3"));
//
//
//        scheduler.add(new Shoot3BallsCommand().withName("Shoot Third 3 Balls"));
//
//        ArrayList<CurvePoint> pathToSpike1A = new ArrayList<>();
//        pathToSpike1A.add(new CurvePoint(12.06, y(11.88), 1.00, 1.00, 5.00, Math.toRadians(60.0), 0.90));
//        pathToSpike1A.add(new CurvePoint(11.88, y(30.42), 1.00, 1.00, 7.00, Math.toRadians(60.0), 0.90));
//        pathToSpike1A.add(new CurvePoint(11.88, y(38.70), 0.80, 0.85, 7.00, Math.toRadians(60.0), 0.90));
//        pathToSpike1A.add(new CurvePoint(11.88, y(52.02), 0.55, 0.85, 2.00, Math.toRadians(60.0), 0.90));
//        scheduler.add(new FollowPathCommand(pathToSpike1A, Math.toRadians(90), debug).withName("Drive to Spike 1A"));
//
//        ArrayList<CurvePoint> pathToShot4 = new ArrayList<>();
//        pathToShot4.add(new CurvePoint(11.88, y(52.02), 1.00, 1.00, 10.00, Math.toRadians(60.0), 0.90));
//        pathToShot4.add(new CurvePoint(43.02, y(21.78), 1.00, 1.00, 10.00, Math.toRadians(60.0), 1.00));
//        scheduler.add(new FollowPathCommand(pathToShot4, mirroredHeading(180), debug, 5)
//                .transitionWhenDistancetoEndIsLessThan(20)
//                .withName("Drive to Shot 4"));
//
//        scheduler.add(new Shoot3BallsCommand().withName("Shoot fourth 3 Balls"));
//        scheduler.add(new EndCommand().withName("End of auto"));
//    }
//    /**
//     * NEAR Variant C: AleTest
//     * Based on Sort version, but shoots preloads directly without sorting by pattern.
//     */
//    public void buildAleTest() {
//        commonNearInit();
//
//        scheduler.add(new SetShooterPreloadModeCommand().withName("set to preload"));
//        scheduler.add(new SeekAndAimCommand(0, alliance).transitionImmediately().withName("Aim at 0 degrees"));
//
//        // 1. Drive to Shooting Spot 1
//        ArrayList<CurvePoint> pathToShoot = new ArrayList<>();
//        boolean debug = false;
//        pathToShoot.add(new CurvePoint(41.04, y(54.54), 0.75, 0.85, 10.00, Math.toRadians(60.0), 0.90));
//        pathToShoot.add(new CurvePoint(40.68, y(46.44), 1.00, 0.95, 10.00, Math.toRadians(60.0), 0.90));
//        pathToShoot.add(new CurvePoint(39.96, y(39.42), 1.00, 0.95, 10.00, Math.toRadians(60.0), 0.90));
//        pathToShoot.add(new CurvePoint(30.60, y(28.98), 1.00, 0.90, 10.00, Math.toRadians(60.0), 0.90));
//        scheduler.add(new FollowPathCommand(pathToShoot, Math.toRadians(-90), debug)
//                .transitionImmediately()
//                .withName("Drive to Shooting Spot 1"));
//
//        // Change: Using Shoot3BallsCommand instead of ShootByPatternCommand
//        scheduler.add(new Shoot3BallsCommand().withName("Shoot first 3 Balls"));
//
//        // 2. Drive to Spike 1
//        ArrayList<CurvePoint> pathToSpike1 = new ArrayList<>();
//        pathToSpike1.add(new CurvePoint(30.60, y(28.98), 0.70, 0.85, 7.00, Math.toRadians(60.0), 0.60));
//        pathToSpike1.add(new CurvePoint(17.28, y(16.56), 0.70, 1.00, 7.00, Math.toRadians(60.0), 0.90));
//        pathToSpike1.add(new CurvePoint(11.88, y(16.74), 0.70, 1.00, 5.00, Math.toRadians(60.0), 0.90));
//        pathToSpike1.add(new CurvePoint(11.88, y(30.42), 0.90, 1.00, 7.00, Math.toRadians(60.0), 0.90));
//        pathToSpike1.add(new CurvePoint(11.88, y(38.70), 0.70, 0.85, 7.00, Math.toRadians(60.0), 0.90));
//        pathToSpike1.add(new CurvePoint(11.88, y(52.02), 0.40, 0.85, 2.00, Math.toRadians(60.0), 0.90));
//        scheduler.add(new FollowPathCommand(pathToSpike1, Math.toRadians(90), debug).withName("Drive to Spike 1"));
//
//        // 3. Drive to Shoot Spot 2
//        ArrayList<CurvePoint> pathToShot2 = new ArrayList<>();
//        pathToShot2.add(new CurvePoint(12.24, y(48.06), 1.00, 0.40, 10.00, Math.toRadians(60.0), 0.60));
//        pathToShot2.add(new CurvePoint(32.76, y(31.86), 1.00, 1.00, 10.00, Math.toRadians(60.0), 1.00));
//        scheduler.add(new FollowPathCommand(pathToShot2, mirroredHeading(180), debug)
//                .withName("Drive to Shoot Spot 2"));
//
//        scheduler.add(new Shoot3BallsCommand().withName("Shoot second 3 Balls"));
//
//        scheduler.add(new EndCommand().withName("End of AleTest"));
//    }
//
//    public void buildFarCornerHusky() {
//        commonFarInit();
//
//        // 1. Preload
//        scheduler.add(new SetShooterPreloadModeCommand().withName("set to preload"));
//
//        scheduler.add(new WaitCommand(2));
//
////        // 2. Drive to Shooting Spot
////        ArrayList<CurvePoint> pathToShoot = new ArrayList<>();
//        boolean debug = false;
////        pathToShoot.add(new CurvePoint(-62.04, y(14.00), 1.00, 1.00, 5, Math.toRadians(60.0), 1.00));
////        pathToShoot.add(new CurvePoint(-54.90, y(11.88), 1.00, 1.00, 5, Math.toRadians(60.0), 1.00));
////        pathToShoot.add(new CurvePoint(-49.00, y(11.88), 0.80, 1.00, 5, Math.toRadians(60.0), 1.00));
////        scheduler.add(new FollowPathCommand(pathToShoot, Math.toRadians(90), debug)
////                .transitionImmediately()
////                .withName("Drive to Shooting Spot"));
//
////        scheduler.add(new WaitCommand(1.5));
//        scheduler.add(new Shoot3BallsCommand().withName("Shoot first 3 Balls"));
//
//        // 3. Drive to Corner
//        ArrayList<CurvePoint> driveToCornerA = new ArrayList<>();
//        driveToCornerA.add(new CurvePoint(-50.70, y(11.70), 1.00, 0.40, 10.00, Math.toRadians(60.0), 0.60));
//        driveToCornerA.add(new CurvePoint(-54.54, y(50.04), 1.00, 0.40, 5.00, Math.toRadians(60.0), 0.60));
//        driveToCornerA.add(new CurvePoint(-58.68, y(62.46), 1.00, 0.40, 5.00, Math.toRadians(60.0), 0.60));
//        double headingA = isRed ? 75 : 115;
//        scheduler.add(new FollowPathCommand(driveToCornerA, Math.toRadians(headingA), debug, 2.5)
//                .withName("Drive In Bump 1"));
//
////        scheduler.add(new WaitCommand(.3));
////
////        ArrayList<CurvePoint> driveToCornerB = new ArrayList<>();
////        driveToCornerB.add(new CurvePoint(-58.68, y(62.46), 1.00, 0.40, 10.00, Math.toRadians(60.0), 0.60));
////        driveToCornerB.add(new CurvePoint(-55.26, y(52.76), 1.00, 0.40, 10.00, Math.toRadians(60.0), 0.60));
////        scheduler.add(new FollowPathCommand(driveToCornerB, Math.toRadians(-90), debug, 2)
////                .withName("Drive Out Bump 1"));
//
////        ArrayList<CurvePoint> driveToCornerC = new ArrayList<>();
////        driveToCornerC.add(new CurvePoint(-55.26, y(52.76), 1.00, 0.40, 10.00, Math.toRadians(60.0), 0.60));
////        driveToCornerC.add(new CurvePoint(-61.02, y(59.76), 1.00, 0.40, 10.00, Math.toRadians(60.0), 0.60));
////        scheduler.add(new FollowPathCommand(driveToCornerC, Math.toRadians(90), debug, 2)
////                .withName("Drive In Bump 2"));
//
////        scheduler.add(new WaitCommand(0.3));
//
////        ArrayList<CurvePoint> driveToCornerD = new ArrayList<>();
////        driveToCornerD.add(new CurvePoint(-61.02, y(59.76), 1.00, 0.40, 10.00, Math.toRadians(60.0), 0.60));
////        driveToCornerD.add(new CurvePoint(-59.02, y(47.14), 1.00, 0.40, 10.00, Math.toRadians(60.0), 0.60));
////        scheduler.add(new FollowPathCommand(driveToCornerD, Math.toRadians(-90), debug, 2)
////                .withName("Drive to Corner D"));
////
////        scheduler.add(new WaitForDrivetrainCommand());
////
////        ArrayList<CurvePoint> driveToCornerE = new ArrayList<>();
////        driveToCornerE.add(new CurvePoint(-59.02, y(47.14), 1.00, 0.40, 10.00, Math.toRadians(60.0), 0.60));
////        driveToCornerE.add(new CurvePoint(-61.02, y(59.76), 1.00, 0.40, 10.00, Math.toRadians(60.0), 0.60));
////        scheduler.add(new FollowPathCommand(driveToCornerE, Math.toRadians(90), debug, 2)
////                .withName("Drive to Corner E"));
////
////        scheduler.add(new WaitCommand(0.3));
//
//        // 4. Drive to Shoot Spot 2
//        ArrayList<CurvePoint> driveToShoot2 = new ArrayList<>();
//        driveToShoot2.add(new CurvePoint(-61.20, y(58.86), 1.00, 0.40, 10.00, Math.toRadians(60.0), 0.60));
//        driveToShoot2.add(new CurvePoint(-58.00, y(21.42), 1.00, 1.00, 10.00, Math.toRadians(60.0), 1.00));
////        driveToShoot2.add(new CurvePoint(-53.64, y(15.48), 1.00, 1.00, 10.00, Math.toRadians(60.0), 1.00));
//        double headingShoot2 = -90;
//        scheduler.add(new FollowPathCommand(driveToShoot2, Math.toRadians(headingShoot2), debug)
//                .withName("Drive to Shoot spot 2"));
//
//        scheduler.add(new Shoot2BallsCommand().withName("Shoot second 2 Balls"));
////        scheduler.add(new Shoot3BallsCommand().withName("Shoot second 3 Balls"));
//
//        // 5. Drive to Spike 1
//        ArrayList<CurvePoint> pathToSpike1 = new ArrayList<>();
//        pathToSpike1.add(new CurvePoint(-49.14, y(11.88), 1.00, 1.00, 10.00, Math.toRadians(60.0), 1.00));
//        pathToSpike1.add(new CurvePoint(-36.00, y(22.14), 1.00, 1.00, 10.00, Math.toRadians(60.0), 1.00));
//        pathToSpike1.add(new CurvePoint(-35.46, y(42.66), 0.80, 0.90, 10.00, Math.toRadians(60.0), 0.60));
//        pathToSpike1.add(new CurvePoint(-35.28, y(51.82), 0.80, 0.90, 10.00, Math.toRadians(60.0), 0.60));
//        scheduler.add(new FollowPathCommand(pathToSpike1, Math.toRadians(90), debug).withName("Drive to Spike 1"));
//
//        // 6. Drive to Shoot Spot 3
//        ArrayList<CurvePoint> pathToShot3 = new ArrayList<>();
//        pathToShot3.add(new CurvePoint(-35.28, y(51.84), 1.00, 1.00, 5.00, Math.toRadians(60.0), 1.00));
//        pathToShot3.add(new CurvePoint(-58.00, y(21.42), 1.00, 1.00, 5.00, Math.toRadians(60.0), 1.00));
////        pathToShot3.add(new CurvePoint(-46.62, y(12.62), 1.00, 1.00, 5.00, Math.toRadians(60.0), 1.00));
//        scheduler.add(new FollowPathCommand(pathToShot3, Math.toRadians(-90), debug)
//                .withName("Drive to Shoot Spot 3"));
//
//        scheduler.add(new Shoot3BallsCommand().withName("Shoot third 3 Balls"));
//
//        // 7. Begin Husky Chase 1
//        scheduler.add(new AutoIntakeCommand((isRed ? 270 : 90), false, 3.0, 0.0, 60.0)
//                .withName("Intake with Husky"));
//
//        // 8. Drive to Shoot 3
//        ArrayList<CurvePoint> driveToShoot3 = new ArrayList<>();
//        driveToShoot3.add(new CurvePoint(-61.20, y(58.86), 1.00, 0.40, 10.00, Math.toRadians(60.0), 0.60));
//        driveToShoot3.add(new CurvePoint(-58.00, y(21.42), 1.00, 1.00, 10.00, Math.toRadians(60.0), 1.00));
////        driveToShoot3.add(new CurvePoint(-53.64, y(15.48), 1.00, 1.00, 10.00, Math.toRadians(60.0), 1.00));
//
//        double headingShoot3 = -90;
//        scheduler.add(new FollowPathCommand(driveToShoot3, Math.toRadians(headingShoot3), debug)
//                .withName("Drive to Shoot spot 3"));
//
//        scheduler.add(new Shoot3BallsCommand().withName("Shoot fourth 3 Balls"));
//
//
//        // 7. Begin Husky Chase 1
//        scheduler.add(new AutoIntakeCommand((isRed ? 270 : 90), false, 3.0, 0.0, 60.0)
//                .withName("Intake with Husky"));
//
//        // 8. Drive to Shoot 3
//        ArrayList<CurvePoint> driveToShoot4 = new ArrayList<>();
//        driveToShoot4.add(new CurvePoint(-61.20, y(58.86), 1.00, 0.40, 10.00, Math.toRadians(60.0), 0.60));
//        driveToShoot4.add(new CurvePoint(-58.00, y(21.42), 1.00, 1.00, 10.00, Math.toRadians(60.0), 1.00));
////        driveToShoot4.add(new CurvePoint(-53.64, y(15.48), 1.00, 1.00, 10.00, Math.toRadians(60.0), 1.00));
//
//        double headingShoot4 = -90;
//        scheduler.add(new FollowPathCommand(driveToShoot4, Math.toRadians(headingShoot4), debug)
//                .withName("Drive to Shoot spot 4"));
//
//        scheduler.add(new Shoot3BallsCommand().withName("Shoot fourth 3 Balls"));
//
//        commonFarTeardown();
//    }
//
    private void commonFarInit() {
        if (startDelay > 0) {
            scheduler.add(new WaitCommand(startDelay).withName("Initial Delay"));
        }

//        shooter.setFlywheelMode(Shooter.FlywheelMode.AUTO);
//        intake.setIntakeRoller(true);

        Pose2D initialPose = new Pose2D(DistanceUnit.INCH, -62.04, y(14.00), AngleUnit.DEGREES, 0.00);
        drivetrain.setPosition(initialPose);
//        turret.setInitialPosition(0);
    }
//
//    private void commonNearInit() {
//        if (startDelay > 0) {
//            scheduler.add(new WaitCommand(startDelay).withName("Initial Delay"));
//        }
//
//        shooter.setFlywheelMode(Shooter.FlywheelMode.AUTO);
//        intake.setIntakeRoller(true);
//
//        double initialHeading = isRed ? 90.00 : -90.00;
//        Pose2D initialPose = new Pose2D(DistanceUnit.INCH, 41.04, y(54.54), AngleUnit.DEGREES, initialHeading);
//        drivetrain.setPosition(initialPose);
//        turret.setInitialPosition(isRed ? 15 : -15);
//    }
//
    private void commonFarTeardown() {
        boolean debug = false;
        ArrayList<CurvePoint> pathToPark = new ArrayList<>();
        pathToPark.add(new CurvePoint(-46.00, y(11.20), 0.9, 0.80, 5, Math.toRadians(60.0), 0.60));
        pathToPark.add(new CurvePoint(-35.28, y(20.52), 0.90, 0.80, 3, Math.toRadians(60.0), 0.60));

        scheduler.addTeardown(new FollowPathCommand(pathToPark, Math.toRadians(90), debug)
                .withName("TEARDOWN - Park Robot"));
        scheduler.addTeardown(new EndCommand().withName("TEARDOWN - End of Auto"));
    }
//
//    public void buildFarCornerHuskyOnly() {
//        commonFarInit();
//
//        // 1. Preload
//        scheduler.add(new SetShooterPreloadModeCommand().withName("set to preload"));
//
//        scheduler.add(new WaitCommand(2));
////        // 2. Drive to Shooting Spot
////        ArrayList<CurvePoint> pathToShoot = new ArrayList<>();
//        boolean debug = false;
////        pathToShoot.add(new CurvePoint(-62.04, y(14.00), 1.00, 1.00, 5, Math.toRadians(60.0), 1.00));
////        pathToShoot.add(new CurvePoint(-54.90, y(11.88), 1.00, 1.00, 5, Math.toRadians(60.0), 1.00));
////        pathToShoot.add(new CurvePoint(-49.00, y(11.88), 0.80, 1.00, 5, Math.toRadians(60.0), 1.00));
////        scheduler.add(new FollowPathCommand(pathToShoot, Math.toRadians(90), debug)
////                .transitionImmediately()
////                .withName("Drive to Shooting Spot"));
//
////        scheduler.add(new WaitCommand(1.5));
//        scheduler.add(new Shoot3BallsCommand().withName("Shoot first 3 Balls"));
//
//        // 3. Drive to Corner
//        ArrayList<CurvePoint> driveToCornerA = new ArrayList<>();
//        driveToCornerA.add(new CurvePoint(-50.70, y(11.70), 1.00, 0.40, 10.00, Math.toRadians(60.0), 0.60));
//        driveToCornerA.add(new CurvePoint(-54.54, y(50.04), 1.00, 0.40, 5.00, Math.toRadians(60.0), 0.60));
//        driveToCornerA.add(new CurvePoint(-58.68, y(62.46), 1.00, 0.40, 5.00, Math.toRadians(60.0), 0.60));
//        double headingA = isRed ? 75 : 115;
//        scheduler.add(new FollowPathCommand(driveToCornerA, Math.toRadians(headingA), debug, 2.5)
//                .withName("Drive In Bump 1"));
//
////        scheduler.add(new WaitCommand(.3));
////
////        ArrayList<CurvePoint> driveToCornerB = new ArrayList<>();
////        driveToCornerB.add(new CurvePoint(-58.68, y(62.46), 1.00, 0.40, 10.00, Math.toRadians(60.0), 0.60));
////        driveToCornerB.add(new CurvePoint(-55.26, y(52.76), 1.00, 0.40, 10.00, Math.toRadians(60.0), 0.60));
////        scheduler.add(new FollowPathCommand(driveToCornerB, Math.toRadians(-90), debug, 2)
////                .withName("Drive Out Bump 1"));
//
////        ArrayList<CurvePoint> driveToCornerC = new ArrayList<>();
////        driveToCornerC.add(new CurvePoint(-55.26, y(52.76), 1.00, 0.40, 10.00, Math.toRadians(60.0), 0.60));
////        driveToCornerC.add(new CurvePoint(-61.02, y(59.76), 1.00, 0.40, 10.00, Math.toRadians(60.0), 0.60));
////        scheduler.add(new FollowPathCommand(driveToCornerC, Math.toRadians(90), debug, 2)
////                .withName("Drive In Bump 2"));
//
////        scheduler.add(new WaitCommand(0.3));
//
////        ArrayList<CurvePoint> driveToCornerD = new ArrayList<>();
////        driveToCornerD.add(new CurvePoint(-61.02, y(59.76), 1.00, 0.40, 10.00, Math.toRadians(60.0), 0.60));
////        driveToCornerD.add(new CurvePoint(-59.02, y(47.14), 1.00, 0.40, 10.00, Math.toRadians(60.0), 0.60));
////        scheduler.add(new FollowPathCommand(driveToCornerD, Math.toRadians(-90), debug, 2)
////                .withName("Drive to Corner D"));
////
////        scheduler.add(new WaitForDrivetrainCommand());
////
////        ArrayList<CurvePoint> driveToCornerE = new ArrayList<>();
////        driveToCornerE.add(new CurvePoint(-59.02, y(47.14), 1.00, 0.40, 10.00, Math.toRadians(60.0), 0.60));
////        driveToCornerE.add(new CurvePoint(-61.02, y(59.76), 1.00, 0.40, 10.00, Math.toRadians(60.0), 0.60));
////        scheduler.add(new FollowPathCommand(driveToCornerE, Math.toRadians(90), debug, 2)
////                .withName("Drive to Corner E"));
////
////        scheduler.add(new WaitCommand(0.3));
//
//        // 4. Drive to Shoot Spot 2
//        ArrayList<CurvePoint> driveToShoot2 = new ArrayList<>();
//        driveToShoot2.add(new CurvePoint(-61.20, y(58.86), 1.00, 0.40, 10.00, Math.toRadians(60.0), 0.60));
//        driveToShoot2.add(new CurvePoint(-58.00, y(21.42), 1.00, 1.00, 10.00, Math.toRadians(60.0), 1.00));
////        driveToShoot2.add(new CurvePoint(-53.64, y(15.48), 1.00, 1.00, 10.00, Math.toRadians(60.0), 1.00));
//        double headingShoot2 = -90;
//        scheduler.add(new FollowPathCommand(driveToShoot2, Math.toRadians(headingShoot2), debug)
//                .withName("Drive to Shoot spot 2"));
//
//        scheduler.add(new Shoot2BallsCommand().withName("Shoot second 2 Balls"));
////        scheduler.add(new Shoot3BallsCommand().withName("Shoot second 3 Balls"));
//
//        // 5. Drive to Spike 1
////        ArrayList<CurvePoint> pathToSpike1 = new ArrayList<>();
////        pathToSpike1.add(new CurvePoint(-49.14, y(11.88), 1.00, 1.00, 10.00, Math.toRadians(60.0), 1.00));
////        pathToSpike1.add(new CurvePoint(-36.00, y(22.14), 1.00, 1.00, 10.00, Math.toRadians(60.0), 1.00));
////        pathToSpike1.add(new CurvePoint(-35.46, y(42.66), 0.80, 0.90, 10.00, Math.toRadians(60.0), 0.60));
////        pathToSpike1.add(new CurvePoint(-35.28, y(51.82), 0.80, 0.90, 10.00, Math.toRadians(60.0), 0.60));
////        scheduler.add(new FollowPathCommand(pathToSpike1, Math.toRadians(90), debug).withName("Drive to Spike 1"));
////
////        // 6. Drive to Shoot Spot 3
////        ArrayList<CurvePoint> pathToShot3 = new ArrayList<>();
//
////        pathToShot3.add(new CurvePoint(-35.28, y(51.84), 1.00, 1.00, 5.00, Math.toRadians(60.0), 1.00));
////        pathToShot3.add(new CurvePoint(-58.00, y(21.42), 1.00, 1.00, 5.00, Math.toRadians(60.0), 1.00));
//////        pathToShot3.add(new CurvePoint(-46.62, y(12.62), 1.00, 1.00, 5.00, Math.toRadians(60.0), 1.00));
////        scheduler.add(new FollowPathCommand(pathToShot3, Math.toRadians(-90), debug)
////                .withName("Drive to Shoot Spot 3"));
////
////        scheduler.add(new Shoot3BallsCommand().withName("Shoot third 3 Balls"));
//
//        // 7. Begin Husky Chase 1
//        scheduler.add(new AutoIntakeCommand((isRed ? 260 : 100), false, 3.0, 0.0, 60.0)
//                .withName("Intake with Husky"));
//
//        // 8. Drive to Shoot 3
//        ArrayList<CurvePoint> driveToShoot3 = new ArrayList<>();
//        driveToShoot3.add(new CurvePoint(-61.20, y(58.86), 1.00, 0.40, 10.00, Math.toRadians(60.0), 0.60));
//        driveToShoot3.add(new CurvePoint(-58.00, y(21.42), 1.00, 1.00, 10.00, Math.toRadians(60.0), 1.00));
////        driveToShoot3.add(new CurvePoint(-53.64, y(15.48), 1.00, 1.00, 10.00, Math.toRadians(60.0), 1.00));
//
//        double headingShoot3 = -90;
//        scheduler.add(new FollowPathCommand(driveToShoot3, Math.toRadians(headingShoot3), debug)
//                .withName("Drive to Shoot spot 3"));
//
//        scheduler.add(new Shoot3BallsCommand().withName("Shoot fourth 3 Balls"));
//
//
//        // 7. Begin Husky Chase 2
//        scheduler.add(new AutoIntakeCommand((isRed ? 260 : 100), false, 3.0, 0.0, 60.0)
//                .withName("Intake with Husky"));
//
//        // 8. Drive to Shoot 3
//        ArrayList<CurvePoint> driveToShoot4 = new ArrayList<>();
//        driveToShoot4.add(new CurvePoint(-61.20, y(58.86), 1.00, 0.40, 10.00, Math.toRadians(60.0), 0.60));
//        driveToShoot4.add(new CurvePoint(-58.00, y(21.42), 1.00, 1.00, 10.00, Math.toRadians(60.0), 1.00));
////        driveToShoot4.add(new CurvePoint(-53.64, y(15.48), 1.00, 1.00, 10.00, Math.toRadians(60.0), 1.00));
//
//        double headingShoot4 = -90;
//        scheduler.add(new FollowPathCommand(driveToShoot4, Math.toRadians(headingShoot4), debug)
//                .withName("Drive to Shoot spot 4"));
//
//        scheduler.add(new Shoot3BallsCommand().withName("Shoot fourth 3 Balls"));
//
//        //9. Begin Husky Chase 3
//        scheduler.add(new AutoIntakeCommand((isRed ? 260 : 100), false, 3.0, 0.0, 60.0)
//                .withName("Intake with Husky"));
//
//        // 8. Drive to Shoot 3
//        ArrayList<CurvePoint> driveToShoot5 = new ArrayList<>();
//        driveToShoot5.add(new CurvePoint(-61.20, y(58.86), 1.00, 0.40, 10.00, Math.toRadians(60.0), 0.60));
//        driveToShoot5.add(new CurvePoint(-58.00, y(21.42), 1.00, 1.00, 10.00, Math.toRadians(60.0), 1.00));
////        driveToShoot3.add(new CurvePoint(-53.64, y(15.48), 1.00, 1.00, 10.00, Math.toRadians(60.0), 1.00));
//
//        double headingShoot5 = -90;
//        scheduler.add(new FollowPathCommand(driveToShoot3, Math.toRadians(headingShoot5), debug)
//                .withName("Drive to Shoot spot 4"));
//
//        scheduler.add(new Shoot3BallsCommand().withName("Shoot fifth 3 Balls"));
//
//        commonFarTeardown();
//    }
}
