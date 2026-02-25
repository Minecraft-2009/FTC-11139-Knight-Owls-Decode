//package org.firstinspires.ftc.teamcode.aold.scrim.autonomi;
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
//@Autonomous(name = "Red Close 12 - New")
//@Configurable
//public class RedClose12New extends NrpOpMode {
//
//    private TelemetryManager panelsTelemetry;
//    private Follower follower;
//    private int pathState = 0;
//    private Paths paths;
//
//    double targetX = 132;
//    double targetY = 132;
//
//    // Optional: timers for delays
//    private ElapsedTime pathTimer = new ElapsedTime(); // Add this field at top!
//
//    boolean pathStarted = false;
//
//    @Override
//    public void runOpMode() {  // ← This replaces init() + loop()
//        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
//
//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(new Pose(125.500, 118.700, Math.toRadians(35))); // Blue close start
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
//            // Dynamic Gradual Hood Angle
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
//
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
//        public PathChain P1_Shootfirst0303;
//        public PathChain P2_IntakeOpen0303;
//        public PathChain P3_OpenGate0300;
//        public PathChain P4_ToTriShoot0603;
//        public PathChain P5_Intake0603;
//        public PathChain P6_ToTriShoot0906;
//        public PathChain P7_Intakelast0906;
//        public PathChain P8_Shootlast1209;
//
//        public Paths(Follower follower) {
//            P1_Shootfirst0303 = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(125.500, 118.700),
//
//                                    new Pose(98.800, 95.400)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(0))
//
//                    .build();
//
//            P2_IntakeOpen0303 = follower.pathBuilder().addPath(
//                            new BezierCurve(
//                                    new Pose(98.800, 95.400),
//                                    new Pose(97.508, 55.651),
//                                    new Pose(91.729, 59.604),
//                                    new Pose(126.200, 59.200)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//
//                    .build();
//
//            P3_OpenGate0300 = follower.pathBuilder().addPath(
//                            new BezierCurve(
//                                    new Pose(126.200, 59.200),
//                                    new Pose(115.577, 67.326),
//                                    new Pose(126.872, 68.595)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//
//                    .build();
//
//            P4_ToTriShoot0603 = follower.pathBuilder().addPath(
//                            new BezierCurve(
//                                    new Pose(126.872, 68.595),
//                                    new Pose(100.171, 66.187),
//                                    new Pose(94.732, 75.782),
//                                    new Pose(99.000, 95.500)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//
//                    .build();
//
//            P5_Intake0603 = follower.pathBuilder().addPath(
//                            new BezierCurve(
//                                    new Pose(99.000, 95.500),
//                                    new Pose(78.826, 80.888),
//                                    new Pose(101.601, 83.765),
//                                    new Pose(127.100, 83.500)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//
//                    .build();
//
//            P6_ToTriShoot0906 = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(127.100, 83.500),
//
//                                    new Pose(99.000, 95.500)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//
//                    .build();
//
//            P7_Intakelast0906 = follower.pathBuilder().addPath(
//                            new BezierCurve(
//                                    new Pose(99.000, 95.500),
//                                    new Pose(80.679, 34.099),
//                                    new Pose(97.186, 34.536),
//                                    new Pose(125.000, 34.900)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//
//                    .build();
//
//            P8_Shootlast1209 = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(125.000, 34.900),
//
//                                    new Pose(88.000, 109.000)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(30))
//
//                    .build();
//        }
//    }
//
////    public static double nearRPM(double distance) {
////        return 0.000106346*Math.pow(distance, 4)  - 0.0173809*Math.pow(distance, 3) + 0.666884*Math.pow(distance, 2) + 23.07246*distance + 1959.92984;
////    }
//
//
//    public void autonomousPathUpdate() {
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
//                if (!pathStarted) {
//                    runShooterAtRPM(3550);
//                    follower.followPath(paths.P1_Shootfirst0303);
//                    pathTimer.reset();
//                    pathStarted = true;
//                }
//
//                if (!follower.isBusy()) {
//                    pathStarted = false;
//                    pathState = 1;
//                }
//                break;
//
//            case 1:
//                if (!follower.isBusy()) {
//                    runShooterAtRPM(3550);
//
//                    if (pathTimer.milliseconds() >= 2500) {
//                        shootThreeBallsOn();
//                    }
//                    if (pathTimer.milliseconds() >= 3200) {
//                        shootThreeBallsOff();
//                        pathState = 2;
//                    }
//                }
//                break;
//
//            case 2:
//                if (!pathStarted) {
//                    follower.followPath(paths.P2_IntakeOpen0303);
//                    pathTimer.reset();
//                    pathStarted = true;
//                }
//                if ((pathTimer.milliseconds() >= 400) && (pathTimer.milliseconds() < 1900)) {
//                    Intake.setPower(-1);
//                    Transfer.setPower(-1);
//                }
//                if (pathTimer.milliseconds() >= 1900) {
//                    Transfer.setPower(0);
//                }
//                if ((!follower.isBusy()) && (pathTimer.milliseconds() >= 2300) ){
//                    Intake.setPower(0);
//                    pathStarted = false;
//                    pathState = 3;
//                }
//
//                break;
//
//            case 3:
//                if (!pathStarted) {
//                    Intake.setPower(0);
//                    follower.followPath(paths.P3_OpenGate0300);
//                    pathTimer.reset();
//                    pathStarted = true;
//                }
//                if (!follower.isBusy()) {
//                    Intake.setPower(0);
//                    pathStarted = false;
//                    pathState = 4;
//                }
//                break;
//
//            case 4:
//                if (!pathStarted) {
//                    follower.followPath(paths.P4_ToTriShoot0603);
//                    pathTimer.reset();
//                    pathStarted = true;
//                }
//                if (pathTimer.milliseconds() >= 500) {
//                    runShooterAtRPM(3550);
//                }
//                if (!follower.isBusy()) {
//                    pathStarted = false;
//                    pathState = 5;
//                }
//                break;
//
//            case 5:
//                if (!follower.isBusy()) {
//                    runShooterAtRPM(3500);
//
//                    if (pathTimer.milliseconds() >= 2500) {
//                        shootThreeBallsOn();
//                    }
//                    if (pathTimer.milliseconds() >= 3200) {
//                        shootThreeBallsOff();
//                        pathState = 6;
//                    }
//                }
//                break;
//
//
//            case 6:
//                if (!pathStarted) {
//                    follower.followPath(paths.P5_Intake0603);
//                    pathTimer.reset();
//                    pathStarted = true;
//                }
//                if ((pathTimer.milliseconds() >= 400) && (pathTimer.milliseconds() < 1900)) {
//                    Intake.setPower(-1);
//                    Transfer.setPower(-1);
//                }
//                if (pathTimer.milliseconds() >= 1900) {
//                    Transfer.setPower(0);
//                }
//                if ((!follower.isBusy()) && (pathTimer.milliseconds() >= 2300) ){
//                    Intake.setPower(0);
//                    pathStarted = false;
//                    pathState = 7;
//                }
//                break;
//
//            case 7:
//                if (!pathStarted) {
//                    follower.followPath(paths.P6_ToTriShoot0906);
//                    pathTimer.reset();
//                    pathStarted = true;
//                }
//                if (pathTimer.milliseconds() >= 50) {
//                    runShooterAtRPM(3550);
//                }
//                if (!follower.isBusy()) {
//                    pathStarted = false;
//                    pathState = 8;
//                }
//                break;
//
//            case 8:
//                if (!follower.isBusy()) {
//                    runShooterAtRPM(3500);
//
//                    if (pathTimer.milliseconds() >= 2500) {
//                        shootThreeBallsOn();
//                    }
//                    if (pathTimer.milliseconds() >= 3200) {
//                        shootThreeBallsOff();
//                        pathState = 9;
//                    }
//                }
//                break;
//
//            case 9:
//                if (!pathStarted) {
//                    follower.followPath(paths.P7_Intakelast0906);
//                    pathTimer.reset();
//                    pathStarted = true;
//                }
//                if ((pathTimer.milliseconds() >= 1000) && (pathTimer.milliseconds() < 2300)) {
//                    Intake.setPower(-1);
//                    Transfer.setPower(-1);
//                }
//                if (pathTimer.milliseconds() >= 2200) {
//                    Transfer.setPower(0);
//                }
//                if ((!follower.isBusy()) && (pathTimer.milliseconds() >= 3200) ){
//                    Intake.setPower(0);
//                    pathStarted = false;
//                    pathState = 10;
//                }
//                break;
//
//            case 10:
//                if (!pathStarted) {
//                    follower.followPath(paths.P8_Shootlast1209);
//                    pathTimer.reset();
//                    pathStarted = true;
//                }
//                if (pathTimer.milliseconds() >= 1200) {
//                    runShooterAtRPM(3500);
//                }
//                if (!follower.isBusy()) {
//                    pathStarted = false;
//                    pathState = 11;
//                }
//                break;
//
//            case 11:
//                if (!follower.isBusy()) {
//                    runShooterAtRPM(3500);
//
//                    if (pathTimer.milliseconds() >= 2500) {
//                        shootThreeBallsOn();
//                    }
//                    if (pathTimer.milliseconds() >= 3200) {
//                        shootThreeBallsOff();
//                        pathState = 12;
//                    }
//                }
//                break;
//
//
//            case 12:
//                if (!follower.isBusy()) {
//                    follower.breakFollowing();
//                    pathState = 99;
//                }
//                break;
//
//            case 99:
//                requestOpModeStop();
//                break;
//
//        }
//    }
//
//}