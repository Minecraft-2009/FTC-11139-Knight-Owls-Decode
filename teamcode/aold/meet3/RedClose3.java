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
//@Autonomous(name = "Red Close  LM3")
//@Configurable
//public class RedClose3 extends NrpOpMode {
//
//    private TelemetryManager panelsTelemetry;
//    private Follower follower;
//    private int pathState = 0;
//    private Paths paths;
//
//    double targetX = 132;
//    double targetY = 132;
//
//
//    // Optional: timers for delays
//    private ElapsedTime pathTimer = new ElapsedTime(); // Add this field at top!
//
//    @Override
//    public void runOpMode() {  // ← This replaces init() + loop()
//        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
//
//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(new Pose(121.000, 125.300, Math.toRadians(37))); // Blue close start
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
//
//
//            // Turret AutoLock
//            Pose currentPose = follower.getPose();
//
//            double robotX = follower.getPose().getX();
//            double robotY = follower.getPose().getY();
//            double robotHeadingRad = currentPose.getHeading();
//            robotHeadingRad = (robotHeadingRad + 2*Math.PI) % (2*Math.PI);
//            double robotHeadingDeg = robotHeadingRad * (180/Math.PI);
//            double turretX = robotX - (2.5 * Math.cos(robotHeadingRad) );
//            double turretY = robotY - (2.5 * Math.sin(robotHeadingRad) );
//            double moddedRobotHeadingRad = robotHeadingRad/(2*Math.PI)-0.25;
//            double pos = 0.75 - (1/(2*Math.PI))*Math.atan((-1*turretY+132)/(-1*turretX+132)) + moddedRobotHeadingRad;
//            if (pos > 1){
//                pos --;
//            }
//            pos = (pos-0.5)*360.0/320.0+0.5;
//            if (pos > 1){
//                pos = 1;
//            }
//            pos = Math.max(0.0, Math.min(1.0, pos));
//
//            TurretL.setPosition(pos);
//            TurretR.setPosition(pos);
//
//
////            / Dynamic Gradual Hood Angle
//            double graduatedDistance = Math.sqrt((targetX-turretX)*(targetX-turretX) + (targetY-turretY)*(targetY-turretY)); // distance
//            double graduatedNearRPM = 0.000106346*Math.pow(graduatedDistance, 4)  - 0.0173809*Math.pow(graduatedDistance, 3) + 0.666884*Math.pow(graduatedDistance, 2) + 23.07246*graduatedDistance + 1959.92984;
//            double graduatedNearAngle = -0.000000214714*Math.pow(graduatedDistance, 4) + 0.0000534017*Math.pow(graduatedDistance, 3) - 0.00472863*Math.pow(graduatedDistance, 2) + 0.172755*graduatedDistance + 1.21858;
//            if (graduatedNearAngle < 0.45) {
//                graduatedNearAngle = 0.45;
//            }
//            Angler.setPosition(graduatedNearAngle);
//
//
//            autonomousPathUpdate();
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
//                                    new Pose(121.600, 125.300),
//
//                                    new Pose(91.000, 100.500)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(37))
//
//                    .build();
//
//            Path2 = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(91.000, 100.500),
//
//                                    new Pose(101.300, 84.300)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(0))
//
//                    .build();
//
//            Path3 = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(101.300, 84.300),
//
//                                    new Pose(122.500, 84.300)
//                            )
//                    ).setConstantHeadingInterpolation(Math.toRadians(0))
//
//                    .build();
//
//            Path4 = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(122.500, 84.300),
//
//                                    new Pose(93.800, 87.500)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-30))
//
//                    .build();
//
//            Path5 = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(93.800, 87.500),
//
//                                    new Pose(101.000, 60.000)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(0))
//
//                    .build();
//
//            Path6 = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(101.000, 60.000),
//
//                                    new Pose(122.500, 60.000)
//                            )
//                    ).setConstantHeadingInterpolation(Math.toRadians(0))
//
//                    .build();
//
//            Path7 = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(122.500, 60.000),
//
//                                    new Pose(85.500, 79.500)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-30))
//
//                    .build();
//
//            Path8 = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(85.500, 79.500),
//
//                                    new Pose(100.600, 35.700)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(0))
//
//                    .build();
//
//            Path9 = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(100.600, 35.700),
//
//                                    new Pose(122.500, 35.700)
//                            )
//                    ).setConstantHeadingInterpolation(Math.toRadians(0))
//
//                    .build();
//
//            Path10 = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(122.500, 35.700),
//
//                                    new Pose(85.500, 79.500)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-30))
//
//                    .build();
//
//            Path11 = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(85.500, 79.500),
//
//                                    new Pose(120.500, 72.000)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(0))
//
//                    .build();
//        }
//    }
//
//    public void autonomousPathUpdate() {
//
//
//        // Graduated information
//        Pose currentPose = follower.getPose();
//        double robotX = follower.getPose().getX();
//        double robotY = follower.getPose().getY();
//        double robotHeadingRad = currentPose.getHeading();
//        robotHeadingRad = (robotHeadingRad + 2*Math.PI) % (2*Math.PI);
//        double robotHeadingDeg = robotHeadingRad * (180/Math.PI);
//        double turretX = robotX - (2.5 * Math.cos(robotHeadingRad) );
//        double turretY = robotY - (2.5 * Math.sin(robotHeadingRad) );
//
//        double graduatedDistance = Math.sqrt((targetX-turretX)*(targetX-turretX) + (targetY-turretY)*(targetY-turretY));
//        // y= 0.000106346x^{4} - 0.0173809x^{3} + 0.666884x^{2} + 23.07246x + 1959.92984 = velocity
//        double graduatedNearRPM = 0.000106346*Math.pow(graduatedDistance, 4)  - 0.0173809*Math.pow(graduatedDistance, 3) + 0.666884*Math.pow(graduatedDistance, 2) + 23.07246*graduatedDistance + 1959.92984;
////
////      // y = -0.000000214714x^{4} + 0.0000534017x^{3} - 0.00472863x^{2} + 0.172755x - 1.21858 = angle
//        double graduatedNearAngle = -0.000000214714*Math.pow(graduatedDistance, 4) + 0.0000534017*Math.pow(graduatedDistance, 3) - 0.00472863*Math.pow(graduatedDistance, 2) + 0.172755*graduatedDistance + 1.21858;
//
//
//        switch (pathState) {
//            case 0:
//                TheJohnAustinHatch.setPosition(0.25);
//                runShooterAtRPM(graduatedNearRPM);
//                follower.followPath(paths.Path1);
//                TheJohnAustinHatch.setPosition(0);
//                pathState = 1;
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
//                    Intake.setPower(-1);
//                    Transfer.setPower(-1);
//                    pathTimer.reset();
//                    follower.followPath(paths.Path3);
//                    pathState = 3;
//                }
//                break;
//
//            case 3: // Intaking while driving + extra time
//
//                if (pathTimer.milliseconds() >= 2000) {
//                    Intake.setPower(0);
//                    Transfer.setPower(0);
//                }
//                if (!follower.isBusy() && pathTimer.milliseconds() >= 2000) {
//                    runShooterAtRPM(graduatedNearRPM);
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
//                    runShooterAtRPM(graduatedNearRPM);
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
//                    runShooterAtRPM(graduatedNearRPM);
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