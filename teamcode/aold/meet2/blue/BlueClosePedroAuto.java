//package org.firstinspires.ftc.teamcode.meet2.blue;
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
//@Autonomous(name = "Blue Close Autonomous", group = "Blue")
//@Configurable
//public class BlueClosePedroAuto extends NrpOpMode {  // ← Changed to LinearOpMode
//
//    private TelemetryManager panelsTelemetry;
//    private Follower follower;
//    private int pathState = 0;
//    private Paths paths;
//
//    // Optional: timers for delays
//    private ElapsedTime pathTimer = new ElapsedTime(); // Add this field at top!
//
//    @Override
//    public void runOpMode() {  // ← This replaces init() + loop()
//        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
//
//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(new Pose(21.507584597432903, 120.81213535589264, Math.toRadians(135))); // Blue close start
//
//        paths = new Paths(follower);
//
//
//        panelsTelemetry.debug("Status", "Initialized - Waiting for start");
//        panelsTelemetry.update(telemetry);
//
//        initControls();
//
//        waitForStart();                   // ← LinearOpMode requirement
//        pathTimer.reset();
//
//        panelsTelemetry.debug("Status", "Auto Started");
//        panelsTelemetry.update(telemetry);
//
//        // Main linear loop
//        while (opModeIsActive()) {
//            follower.update();            // ALWAYS call this every iteration
//            autonomousPathUpdate();
//
//            // Turret AutoLock (exactly the same as before)
//            double robotX = follower.getPose().getX();
//            double robotY = follower.getPose().getY();
//            double heading = follower.getPose().getHeading();
//            double turretOffsetX = -1.25;   // inches left of center
//            double turretOffsetY = -3.125;  // inches behind center
//            double turretX = robotX + turretOffsetX * Math.cos(heading) - turretOffsetY * Math.sin(heading);
//            double turretY = robotY + turretOffsetX * Math.sin(heading) + turretOffsetY * Math.cos(heading);
//            double targetX = 24.0;
//            double targetY = 132.0;
//            double dx = targetX - turretX;
//            double dy = targetY - turretY;
//            double angleToTarget = Math.atan2(dy, dx);
//            double errorRad = angleToTarget - heading;
//            while (errorRad > Math.PI) errorRad -= 2 * Math.PI;
//            while (errorRad <= -Math.PI) errorRad += 2 * Math.PI;
//            double turretServoPos = 0.733 - Math.toDegrees(errorRad) / 120.0;   // 120° total range
//            turretServoPos = Math.max(0.0, Math.min(1.0, turretServoPos));
//            Turret.setPosition(turretServoPos);
//
//            // Telemetry (unchanged)
//            panelsTelemetry.debug("Path State", pathState);
//            panelsTelemetry.debug("X", String.format("%.2f", follower.getPose().getX()));
//            panelsTelemetry.debug("Y", String.format("%.2f", follower.getPose().getY()));
//            panelsTelemetry.debug("Heading °", String.format("%.1f", Math.toDegrees(follower.getPose().getHeading())));
//            panelsTelemetry.update(telemetry);
//
//            telemetry.update();
//        }
//    }
//
//    public static class Paths {
//        // ← Everything inside Paths class is 100% identical
//        public PathChain line1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10, Path11;
//
//        public Paths(Follower follower) {
//            line1 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierLine(new Pose(21.508, 120.812), new Pose(51.000, 84.000))
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(140))
//                    .build();
//
//            Path2 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierLine(new Pose(51.000, 84.000), new Pose(40.900, 82.000))
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(180))
//                    .build();
//
//            Path3 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierLine(new Pose(40.900, 82.000), new Pose(20.000, 82.000))
//                    )
//                    .setConstantHeadingInterpolation(Math.toRadians(180))
//                    .build();
//
//            Path4 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierLine(new Pose(20.000, 82.000), new Pose(43.100, 91.900))
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
//                    .build();
//
//            Path5 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierLine(new Pose(43.100, 91.900), new Pose(41.600, 59.000))
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(180))
//                    .build();
//
//            Path6 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierLine(new Pose(41.600, 59.000), new Pose(20.000, 59.000))
//                    )
//                    .setConstantHeadingInterpolation(Math.toRadians(180))
//                    .build();
//
//            Path7 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierLine(new Pose(20.000, 59.000), new Pose(56.300, 81.400))
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
//                    .build();
//
//            Path8 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierLine(new Pose(56.300, 81.400), new Pose(41.600, 34.900))
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(180))
//                    .build();
//
//            Path9 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierLine(new Pose(41.600, 34.900), new Pose(20.000, 34.900))
//                    )
//                    .setConstantHeadingInterpolation(Math.toRadians(180))
//                    .build();
//
//            Path10 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierLine(new Pose(20.000, 34.900), new Pose(56.300, 81.400))
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
//                    .build();
//
//            Path11 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierLine(new Pose(56.300, 81.400), new Pose(36.000, 72.000))
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(180))
//                    .build();
//        }
//    }
//
//
//    public void autonomousPathUpdate() {
//        switch (pathState) {
//            case 0:
//                runShooterAtRPM(NEAR_AUTO_RPM);
//                follower.followPath(paths.line1);
//                pathState = 1;
//                break;
//
//            case 1: // Finished line1 → wait 0.5s → shoot → go to stack
//                if (!follower.isBusy()) {
//                    if (pathTimer.milliseconds() == 0) pathTimer.reset(); // first entry
//                    if (pathTimer.milliseconds() >= 500) {
//                        shootThreeBalls();
//                        stopShooter();
//                        follower.followPath(paths.Path2);
//                        pathState = 2;
//                    }
//                }
//                break;
//
//            case 2: // Go pick up first 3 rings
//                if (!follower.isBusy()) {
//                    Intake.setPower(-1);
//                    pathTimer.reset();
//                    follower.followPath(paths.Path3);
//                    pathState = 3;
//                }
//                break;
//
//            case 3: // Intaking while driving + extra time
//                if (pathTimer.milliseconds() >= 2000) { // ~2 seconds total intake time
//                    Intake.setPower(0);
//                }
//                if (!follower.isBusy() && pathTimer.milliseconds() >= 2000) {
//                    runShooterAtRPM(NEAR_AUTO_RPM);
//                    follower.followPath(paths.Path4);
//                    pathState = 4;
//                }
//                break;
//
//            case 4: // Second shoot
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
//            case 6: // Third intake
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
//            case 8: // Third shoot
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
//            case 10: // Fourth intake (optional?)
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
//            case 12: // Final shoot
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
//            case 14: // Park
//                if (!follower.isBusy()) {
//                    follower.breakFollowing();
//                    pathState = 99;
//                }
//                break;
//
//            case 99:
//                // Done
//                requestOpModeStop();
//                break;
//        }
//    }
//}