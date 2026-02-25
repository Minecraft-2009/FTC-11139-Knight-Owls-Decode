//package org.firstinspires.ftc.teamcode.meet2.red;
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.NrpOpMode;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//
//@Autonomous(name = "Red Close Autonomous", group="Red")
//@Configurable
//public class RedClosePedroAuto extends NrpOpMode {
//
//    private TelemetryManager panelsTelemetry;
//    private Follower follower;
//    private int pathState = 0;
//    private Paths paths;
//    private ElapsedTime pathTimer = new ElapsedTime();
//
//    @Override
//    public void runOpMode() {
//        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
//        follower = Constants.createFollower(hardwareMap);
//
//        // Red close starting pose (mirrored from Blue)
//        follower.setStartingPose(new Pose(122.4924154025671, 120.81213535589264, Math.toRadians(45)));
//
//        paths = new Paths(follower);
//
//        panelsTelemetry.debug("Status", "Initialized - Waiting for start");
//        panelsTelemetry.update(telemetry);
//        initControls();
//
//        waitForStart();
//        pathTimer.reset();
//        panelsTelemetry.debug("Status", "Auto Started");
//        panelsTelemetry.update(telemetry);
//
//        while (opModeIsActive()) {
//            follower.update();
//            autonomousPathUpdate();
//
//            // Turret AutoLock - now aiming at Red goal (141, 130)
//            double robotX = follower.getPose().getX();
//            double robotY = follower.getPose().getY();
//            double heading = follower.getPose().getHeading();
//
//            double turretOffsetX = -1.25;   // inches left of center (from robot POV)
//            double turretOffsetY = -3.125;  // inches behind center
//
//            double turretX = robotX + turretOffsetX * Math.cos(heading) - turretOffsetY * Math.sin(heading);
//            double turretY = robotY + turretOffsetX * Math.sin(heading) + turretOffsetY * Math.cos(heading);
//
//            double targetX = 141.0;  // Red high goal
//            double targetY = 130.0;
//
//            double dx = targetX - turretX;
//            double dy = targetY - turretY;
//            double angleToTarget = Math.atan2(dy, dx);
//
//            double errorRad = angleToTarget - heading;
//            while (errorRad > Math.PI) errorRad -= 2 * Math.PI;
//            while (errorRad <= -Math.PI) errorRad += 2 * Math.PI;
//
//            double turretServoPos = 0.733 - Math.toDegrees(errorRad) / 120.0;   // 120° total range
//            turretServoPos = Math.max(0.0, Math.min(1.0, turretServoPos));
//            Turret.setPosition(turretServoPos);
//
//            // Telemetry
//            panelsTelemetry.debug("Path State", pathState);
//            panelsTelemetry.debug("X", String.format("%.2f", follower.getPose().getX()));
//            panelsTelemetry.debug("Y", String.format("%.2f", follower.getPose().getY()));
//            panelsTelemetry.debug("Heading °", String.format("%.1f", Math.toDegrees(follower.getPose().getHeading())));
//            panelsTelemetry.update(telemetry);
//            telemetry.update();
//        }
//    }
//
//    public static class Paths {
//        public PathChain line1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10, Path11;
//
//        public Paths(Follower follower) {
//
//            line1 = follower
//                    .pathBuilder()
//                    .addPath(new BezierLine(new Pose(122.492, 120.812), new Pose(93.000, 84.000)))
//                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(30))
//                    .build();
//
//            Path2 = follower
//                    .pathBuilder()
//                    .addPath(new BezierLine(new Pose(93.000, 84.000), new Pose(103.100, 82.000)))
//                    .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(0))
//                    .build();
//
//            Path3 = follower
//                    .pathBuilder()
//                    .addPath(new BezierLine(new Pose(103.100, 82.000), new Pose(124.000, 82.000)))
//                    .setConstantHeadingInterpolation(Math.toRadians(0))
//                    .build();
//
//            Path4 = follower
//                    .pathBuilder()
//                    .addPath(new BezierLine(new Pose(124.000, 82.000), new Pose(100.900, 91.900)))
//                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(30))
//                    .build();
//
//            Path5 = follower
//                    .pathBuilder()
//                    .addPath(new BezierLine(new Pose(100.900, 91.900), new Pose(102.400, 59.000)))
//                    .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(0))
//                    .build();
//
//            Path6 = follower
//                    .pathBuilder()
//                    .addPath(new BezierLine(new Pose(102.400, 59.000), new Pose(124.000, 59.000)))
//                    .setConstantHeadingInterpolation(Math.toRadians(0))
//                    .build();
//
//            Path7 = follower
//                    .pathBuilder()
//                    .addPath(new BezierLine(new Pose(124.000, 59.000), new Pose(87.700, 81.400)))
//                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(30))
//                    .build();
//
//            Path8 = follower
//                    .pathBuilder()
//                    .addPath(new BezierLine(new Pose(87.700, 81.400), new Pose(102.400, 34.900)))
//                    .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(0))
//                    .build();
//
//            Path9 = follower
//                    .pathBuilder()
//                    .addPath(new BezierLine(new Pose(102.400, 34.900), new Pose(124.000, 34.900)))
//                    .setConstantHeadingInterpolation(Math.toRadians(0))
//                    .build();
//
//            Path10 = follower
//                    .pathBuilder()
//                    .addPath(new BezierLine(new Pose(124.000, 34.900), new Pose(87.700, 81.400)))
//                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(30))
//                    .build();
//
//            Path11 = follower
//                    .pathBuilder()
//                    .addPath(new BezierLine(new Pose(87.700, 81.400), new Pose(108.000, 72.000)))
//                    .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(0))
//                    .build();
//        }
//    }
//
//    public void autonomousPathUpdate() {
//        switch (pathState) {
//            case 0:
//                runShooterAtRPM(NEAR_AUTO_RPM);
//                follower.followPath(paths.line1);
//                pathState = 1;
//                break;
//
//            case 1:
//                if (!follower.isBusy()) {
//                    if (pathTimer.milliseconds() == 0) pathTimer.reset();
//                    if (pathTimer.milliseconds() >= 500) {
//                        shootThreeBalls();
//                        stopShooter();
//                        follower.followPath(paths.Path2);
//                        pathState = 2;
//                    }
//                }
//                break;
//
//            case 2:
//                if (!follower.isBusy()) {
//                    Intake.setPower(-1);
//                    pathTimer.reset();
//                    follower.followPath(paths.Path3);
//                    pathState = 3;
//                }
//                break;
//
//            case 3:
//                if (pathTimer.milliseconds() >= 2000) {
//                    Intake.setPower(0);
//                }
//                if (!follower.isBusy() && pathTimer.milliseconds() >= 2000) {
//                    runShooterAtRPM(NEAR_AUTO_RPM);
//                    follower.followPath(paths.Path4);
//                    pathState = 4;
//                }
//                break;
//
//            case 4:
//                if (!follower.isBusy()) {
//                    pathTimer.reset();
//                    pathState = 5;
//                }
//                break;
//
//            case 5:
//                if (pathTimer.milliseconds() >= 800) {
//                    shootThreeBalls();
//                    stopShooter();
//                    follower.followPath(paths.Path5);
//                    pathState = 6;
//                }
//                break;
//
//            case 6:
//                if (!follower.isBusy()) {
//                    Intake.setPower(-1);
//                    pathTimer.reset();
//                    follower.followPath(paths.Path6);
//                    pathState = 7;
//                }
//                break;
//
//            case 7:
//                if (pathTimer.milliseconds() >= 1800) Intake.setPower(0);
//                if (!follower.isBusy() && pathTimer.milliseconds() >= 1800) {
//                    runShooterAtRPM(NEAR_AUTO_RPM);
//                    follower.followPath(paths.Path7);
//                    pathState = 8;
//                }
//                break;
//
//            case 8:
//                if (!follower.isBusy()) {
//                    pathTimer.reset();
//                    pathState = 9;
//                }
//                break;
//
//            case 9:
//                if (pathTimer.milliseconds() >= 600) {
//                    shootThreeBalls();
//                    stopShooter();
//                    follower.followPath(paths.Path8);
//                    pathState = 10;
//                }
//                break;
//
//            case 10:
//                if (!follower.isBusy()) {
//                    Intake.setPower(-1);
//                    pathTimer.reset();
//                    follower.followPath(paths.Path9);
//                    pathState = 11;
//                }
//                break;
//
//            case 11:
//                if (pathTimer.milliseconds() >= 1500) Intake.setPower(0);
//                if (!follower.isBusy() && pathTimer.milliseconds() >= 1500) {
//                    runShooterAtRPM(NEAR_AUTO_RPM);
//                    follower.followPath(paths.Path10);
//                    pathState = 12;
//                }
//                break;
//
//            case 12:
//                if (!follower.isBusy()) {
//                    pathTimer.reset();
//                    pathState = 13;
//                }
//                break;
//
//            case 13:
//                if (pathTimer.milliseconds() >= 600) {
//                    shootThreeBalls();
//                    stopShooter();
//                    follower.followPath(paths.Path11);
//                    pathState = 14;
//                }
//                break;
//
//            case 14:
//                if (!follower.isBusy()) {
//                    follower.breakFollowing();
//                    pathState = 99;
//                }
//                break;
//
//            case 99:
//                requestOpModeStop();
//                break;
//        }
//    }
//}