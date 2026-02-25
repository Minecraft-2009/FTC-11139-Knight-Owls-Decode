//package org.firstinspires.ftc.teamcode.aold.meet3;
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
//@Autonomous(name = "Blue --- NO TURRET ---")
//@Configurable
//public class BlueNoTurret extends NrpOpMode {
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
//        follower.setStartingPose(new Pose(22.400, 125.300, Math.toRadians(143))); // Blue close start
//
//        paths = new Paths(follower);
//
//
//        panelsTelemetry.debug("Status", "Initialized - Waiting for start");
//        panelsTelemetry.update(telemetry);
//
//        initControls();
//
//        waitForStart();
//        pathTimer.reset();
//
//        panelsTelemetry.debug("Status", "Auto Started");
//        panelsTelemetry.update(telemetry);
//
//        // Main linear loop
//        while (opModeIsActive()) {
//            follower.update();  // necessary
//            autonomousPathUpdate();
//
//            // Turret AutoLock (exactly the same as before)
//            double robotX = follower.getPose().getX();
//            double robotY = follower.getPose().getY();
//            double robotHeadingDeg = Math.toDegrees(follower.getPose().getHeading());
//            robotHeadingDeg = (robotHeadingDeg + 360) % 360;
//            double targetX = 0;
//            double targetY = 144;
//            double dx = targetX - robotX;
//            double dy = targetY - robotY;
//            double angleToTargetDeg = Math.toDegrees(Math.atan2(dy, dx));
//            angleToTargetDeg = (angleToTargetDeg + 360) % 360;
//            double turretAngleDeg = angleToTargetDeg - robotHeadingDeg;
//            turretAngleDeg = ((turretAngleDeg + 540) % 360) - 180;
//            double TURRET_RANGE_DEG = 340.0;
//            double HALF_RANGE = TURRET_RANGE_DEG / 2.0;
//            turretAngleDeg = Math.max(-HALF_RANGE, Math.min(HALF_RANGE, turretAngleDeg));
//            double pos = 0.3 - (turretAngleDeg / TURRET_RANGE_DEG);
//            double servoCenter = 0.5;
//            double servoPerDegree = 0.00325;
//            pos = Math.max(0.0, Math.min(1.0, pos));
//            TurretL.setPosition(pos);
//            TurretR.setPosition(pos);
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
//        public PathChain Path1;
//        public PathChain Path2;
//        public PathChain Path3;
//        public PathChain Path4;
//        public PathChain Path5;
//        public PathChain Path6;
//        public PathChain Path7;
//        public PathChain Path8;
//        public PathChain Path9;
//        public PathChain Path10;
//        public PathChain Path11;
//
//        public Paths(Follower follower) {
//            Path1 = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(22.400, 125.300),
//
//                                    new Pose(48.000, 104.900)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(143))
//
//                    .build();
//
//            Path2 = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(48.000, 104.900),
//
//                                    new Pose(42.500, 84.300)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180))
//
//                    .build();
//
//            Path3 = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(42.500, 84.300),
//
//                                    new Pose(21.500, 84.300)
//                            )
//                    ).setConstantHeadingInterpolation(Math.toRadians(180))
//
//                    .build();
//
//            Path4 = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(21.500, 84.300),
//
//                                    new Pose(49.000, 87.500)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(126))
//
//                    .build();
//
//            Path5 = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(49.000, 87.500),
//
//                                    new Pose(42.500, 60.000)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(126), Math.toRadians(180))
//
//                    .build();
//
//            Path6 = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(42.500, 60.000),
//
//                                    new Pose(21.500, 60.000)
//                            )
//                    ).setConstantHeadingInterpolation(Math.toRadians(180))
//
//                    .build();
//
//            Path7 = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(21.500, 60.000),
//
//                                    new Pose(59.000, 79.500)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))
//
//                    .build();
//
//            Path8 = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(59.000, 79.500),
//
//                                    new Pose(42.500, 35.700)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))
//
//                    .build();
//
//            Path9 = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(42.500, 35.700),
//
//                                    new Pose(21.500, 35.700)
//                            )
//                    ).setConstantHeadingInterpolation(Math.toRadians(180))
//
//                    .build();
//
//            Path10 = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(21.500, 35.700),
//
//                                    new Pose(59.000, 79.500)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))
//
//                    .build();
//
//            Path11 = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(59.000, 79.500),
//
//                                    new Pose(21.000, 72.000)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))
//
//                    .build();
//        }
//    }
//
//    public void autonomousPathUpdate() {
//        switch (pathState) {
//            case 0:
//                TurretL.setPosition(0.5);
//                TurretR.setPosition(0.5);
//                TheJohnAustinHatch.setPosition(0.25);
//                runShooterAtRPM(NEAR_AUTO_RPM);
//                follower.followPath(paths.Path1);
//                TheJohnAustinHatch.setPosition(0);
//                break;
//
//            case 1:
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
//            case 2: // Go pick up first 3 balls
//                if (!follower.isBusy()) {
//                    TheJohnAustinHatch.setPosition(0);
//                    Intake.setPower(-1);
//                    Transfer.setPower(-1);
//                    pathTimer.reset();
//                    follower.followPath(paths.Path3);
//                    pathState = 3;
//                }
//                break;
//
//            case 3: // Intaking while driving + extra time
//                if (pathTimer.milliseconds() >= 2000) {
//                    Intake.setPower(0);
//                    Transfer.setPower(0);
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
//                    TheJohnAustinHatch.setPosition(0.25);
//                    shootThreeBalls();
//                    stopShooter();
//                    TheJohnAustinHatch.setPosition(0);
//                    follower.followPath(paths.Path5);
//                    pathState = 6;
//                }
//                break;
//
//            case 6: // Third intake
//                if (!follower.isBusy()) {
//                    Intake.setPower(-1);
//                    Transfer.setPower(-1);
//                    pathTimer.reset();
//                    follower.followPath(paths.Path6);
//                    pathState = 7;
//                }
//                break;
//
//            case 7:
//                if (pathTimer.milliseconds() >= 1800) {
//                    Intake.setPower(0);
//                    Transfer.setPower(0);
//                }
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
//                if (pathTimer.milliseconds() >= 400) {
//                    TheJohnAustinHatch.setPosition(0.25);
//                    shootThreeBalls();
//                    stopShooter();
//                    TheJohnAustinHatch.setPosition(0);
//                    follower.followPath(paths.Path8);
//                    pathState = 10;
//                }
//                break;
//
//            case 10:
//                if (!follower.isBusy()) {
//                    Intake.setPower(-1);
//                    Transfer.setPower(-1);
//                    pathTimer.reset();
//                    follower.followPath(paths.Path9);
//                    pathState = 11;
//                }
//                break;
//
//            case 11:
//                if (pathTimer.milliseconds() >= 1500) {
//                    Intake.setPower(0);
//                    Transfer.setPower(0);
//                }
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
//                    TheJohnAustinHatch.setPosition(0.25);
//                    shootThreeBalls();
//                    stopShooter();
//                    TheJohnAustinHatch.setPosition(0);
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