//package org.firstinspires.ftc.teamcode.meet2.red;
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.NrpOpMode;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//
//@Autonomous(name = "Red Meta Autonomous", group="Red")
//@Configurable
//public class RedMetaPedroAuto extends NrpOpMode {
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
//        follower.setStartingPose(new Pose(88.2, 8.0, Math.toRadians(0)));
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
//            // Optional: telemetry for debugging
//            telemetry.addData("Path State", pathState);
//            telemetry.addData("Follower Busy", follower.isBusy());
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
//
//        public PathChain line1;
//        public PathChain Path2;
//        public PathChain Path3;
//        public PathChain Path4;
//        public PathChain Path5;
//        public PathChain Path6;
//        public PathChain Path7;
//        public PathChain Path8;
//        public PathChain Path9;
//        public PathChain Path10;
//
//        public Paths(Follower follower) {
//            line1 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierCurve(
//                                    new Pose(88.200, 8.000),
//                                    new Pose(86.000, 15.200),
//                                    new Pose(133.800, 11.000)
//                            )
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//                    .build();
//
//            Path2 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierLine(new Pose(133.800, 11.000), new Pose(133.810, 10.888))
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
//                    .build();
//
//            Path3 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierLine(new Pose(133.800, 11.000), new Pose(91.700, 14.400))
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(30))
//                    .build();
//
//            Path4 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierLine(new Pose(91.700, 14.400), new Pose(133.800, 11.000))
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(0))
//                    .build();
//
//            Path5 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierLine(new Pose(133.800, 11.000), new Pose(90.000, 11.000))
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(30))
//                    .build();
//
//            Path6 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierLine(new Pose(90.000, 11.000), new Pose(133.000, 9.000))
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(0))
//                    .build();
//
//            Path7 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierLine(new Pose(133.000, 9.000), new Pose(90.000, 11.000))
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(30))
//                    .build();
//
//            Path8 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierLine(new Pose(90.000, 11.000), new Pose(133.000, 11.000))
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(0))
//                    .build();
//
//            Path9 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierLine(new Pose(133.000, 11.000), new Pose(90.000, 11.000))
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(30))
//                    .build();
//
//            Path10 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierLine(new Pose(90.000, 11.000), new Pose(110.000, 12.000))
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(0))
//                    .build();
//        }
//    }
//
//    public void autonomousPathUpdate() {
//        // Critical debug line — remove later if you want
//        telemetry.addData("State", pathState);
//
//        switch (pathState) {
//
//            case 0:
//                runShooterAtRPM(FAR_TARGET_RPM);  // or your value
//                pathTimer.reset();
//                pathState = 1;
//                break;
//
//            case 1:
//                if (pathTimer.milliseconds() >= 2000) {
//                    shootThreeBalls();
//                    stopShooter();
//                    pathTimer.reset();
//                    pathState = 2;
//                }
//                break;
//
//            case 2:
//                if (pathTimer.milliseconds() >= 300) {
//                    Intake.setPower(-1);
//                    follower.followPath(paths.line1);
//                    pathState = 3;
//                }
//                break;
//
//            case 3:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.Path2);
//                    pathState = 4;  // ← This line runs
//                }
//                break;
//
//            // THIS WILL NOW BE REACHED BECAUSE autonomousPathUpdate() runs every loop
//            case 4:
//                if (!follower.isBusy()) {
//                    Intake.setPower(0);
//                    runShooterAtRPM(FAR_TARGET_RPM);
//                    follower.followPath(paths.Path3);
//                    pathTimer.reset();
//                    pathState = 5;
//                }
//                break;
//
//            case 5:
//                if (!follower.isBusy() && pathTimer.milliseconds() >= 3300) {
//                    shootThreeBalls();
//                    stopShooter();
//                    pathTimer.reset();
//                    pathState = 6;
//                }
//                break;
//
//            // 6: Start intake + begin next intake run
//            case 6:
//                if (pathTimer.milliseconds() >= 400) {
//                    Intake.setPower(-1);
//                    follower.followPath(paths.Path4);
//                    pathState = 7;
//                }
//                break;
//
//
//            // 9: Done intaking 2nd stack → stop intake, spin up, go to shoot (Path7)
//            case 7:
//                if (!follower.isBusy()) {
//                    Intake.setPower(0);
//                    runShooterAtRPM(FAR_TARGET_RPM);
//                    follower.followPath(paths.Path5);
//                    pathTimer.reset();
//                    pathState = 8;
//                }
//                break;
//
//            // 10: Shoot 3rd set
//            case 8:
//                if (!follower.isBusy() && pathTimer.milliseconds() >= 3300) {
//                    shootThreeBalls();
//                    stopShooter();
//                    pathTimer.reset();
//                    pathState = 9;
//                }
//                break;
////commented out
////            // 11: Intake 3rd stack
////            case 9:
////                if (pathTimer.milliseconds() >= 400) {
////                    Intake.setPower(-1);
////                    follower.followPath(paths.Path6);
////                    pathState = 10;
////                }
////                break;
////
////
////            // 14: Stop intake, spin up, go to shoot (Path11)
////            case 10:
////                if (!follower.isBusy()) {
////                    Intake.setPower(0);
////                    runShooterAtRPM(FAR_TARGET_RPM);
////                    follower.followPath(paths.Path7);
////                    pathTimer.reset();
////                    pathState = 11;
////                }
////                break;
////
////            // 15: Shoot 4th set
////            case 11:
////                if (!follower.isBusy() && pathTimer.milliseconds() >= 3300) {
////                    shootThreeBalls();
////                    stopShooter();
////                    pathTimer.reset();
////                    pathState = 12;
////                }
////                break;
//
//            // 18: Final shoot
////            case 9:
////                if (!follower.isBusy() && pathTimer.milliseconds() >= 3300) {
////                    shootThreeBalls();
////                    stopShooter();
////                    pathTimer.reset();
////                    pathState = 13;
////                }
////                break;
//
//            // 19: Park (Path14)
//            case 9:
//                if (pathTimer.milliseconds() >= 400) {
//                    follower.followPath(paths.Path10);
//                    pathState = 14;
//                }
//                break;
//
//            case 10:
//                if (!follower.isBusy()) {
//                    follower.breakFollowing();
//                    pathState = 99;
//                }
//                break;
//
//            // 20: Done
//            case 99:
//                requestOpModeStop();
//                break;
//        }
//    }}