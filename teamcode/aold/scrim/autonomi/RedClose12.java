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
//@Autonomous(name = "Red Close 12")
//@Configurable
//public class RedClose12 extends NrpOpMode {
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
//        public PathChain P1_ShootfirstIntake33;
//        public PathChain P2_ToTriShoot66;
//        public PathChain P3_OpenGateIntake60;
//        public PathChain P4_ToTriShoot93;
//        public PathChain P5_OpenGateIntake90;
//        public PathChain P6_ToTriShoot123;
//        public PathChain P7_Intake1strow123;
//        public PathChain P8_Shootlastballs156;
//
//        public Paths(Follower follower) {
//            P1_ShootfirstIntake33 = follower.pathBuilder().addPath(
//                            new BezierCurve(
//                                    new Pose(125.500, 118.700),
//                                    new Pose(104.681, 100.469),
//                                    new Pose(81.214, 95.016),
//                                    new Pose(94.084, 75.082),
//                                    new Pose(86.140, 54.712),
//                                    new Pose(99.189, 59.478),
//                                    new Pose(126.200, 59.400)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(0))
//
//                    .build();
//
//            P2_ToTriShoot66 = follower.pathBuilder().addPath(
//                            new BezierCurve(
//                                    new Pose(126.200, 59.400),
//                                    new Pose(104.028, 70.963),
//                                    new Pose(86.300, 87.100)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(20))
//
//                    .build();
//
//            P3_OpenGateIntake60 = follower.pathBuilder().addPath(
//                            new BezierCurve(
//                                    new Pose(86.300, 87.100),
//                                    new Pose(102.692, 62.096),
//                                    new Pose(131.600, 60.200)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(20), Math.toRadians(40))
//
//                    .build();
//
//            P4_ToTriShoot93 = follower.pathBuilder().addPath(
//                            new BezierCurve(
//                                    new Pose(131.600, 60.200),
//                                    new Pose(104.036, 69.129),
//                                    new Pose(86.200, 83.100)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(20))
//
//                    .build();
//
//            P5_OpenGateIntake90 = follower.pathBuilder().addPath(
//                            new BezierCurve(
//                                    new Pose(86.200, 83.100),
//                                    new Pose(110.548, 56.264),
//                                    new Pose(131.000, 60.800)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(20), Math.toRadians(40))
//
//                    .build();
//
//            P6_ToTriShoot123 = follower.pathBuilder().addPath(
//                            new BezierCurve(
//                                    new Pose(131.000, 60.800),
//                                    new Pose(102.144, 67.673),
//                                    new Pose(86.300, 82.900)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(20))
//
//                    .build();
//
//            P7_Intake1strow123 = follower.pathBuilder().addPath(
//                            new BezierCurve(
//                                    new Pose(86.300, 82.900),
//                                    new Pose(106.982, 79.407),
//                                    new Pose(125.900, 83.600)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(20), Math.toRadians(0))
//
//                    .build();
//
//            P8_Shootlastballs156 = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(125.900, 83.600),
//
//                                    new Pose(91.000, 117.500)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(35))
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
//                    runShooterAtRPM(graduatedNearRPM);
//                    follower.followPath(paths.P1_ShootfirstIntake33);
//                    pathTimer.reset();           // Always safe here — only once per path
//                    pathStarted = true;
//                }
//                if ((pathTimer.milliseconds() >= 700) && (pathTimer.milliseconds() < 1600)) {
//                    shootThreeBallsOn();
//                    telemetry.addData("Auto Status", "Path 1, shooting");
//                }
//                if ((pathTimer.milliseconds() >= 1600) && (pathTimer.milliseconds() < 2500)) {
//                    shootThreeBallsOff();
//                    stopShooter();
//                    telemetry.addData("Auto Status", "Path 1, stopped shooting");
//                }
//                if ((pathTimer.milliseconds() >= 2500) && (pathTimer.milliseconds() < 3000)) {
//                    Intake.setPower(-1);
//                    Transfer.setPower(-1);
//                }
//                if (pathTimer.milliseconds() >= 3500) {
//                    Transfer.setPower(0);
//                }
//                if (!follower.isBusy()) {
//                    Intake.setPower(0);
//                    pathStarted = false;
//                    pathState = 1;
//                }
//                break;
//
//            case 1:
//                if (!pathStarted) {
//                    follower.followPath(paths.P2_ToTriShoot66);
//                    pathTimer.reset();
//                    pathStarted = true;
//                }
//
//                if (pathTimer.milliseconds() >= 1000) {
//                    runShooterAtRPM(graduatedNearRPM);
//                }
//                if (!follower.isBusy()) {
//                    pathStarted = false;
//                    pathState = 2;
//                }
//                break;
//
//            case 2:
//                if ((pathTimer.milliseconds() >= 0) && (pathTimer.milliseconds() < 1000)) {
//                    shootThreeBallsOn();
//                }
//                if (pathTimer.milliseconds() >= 1000) {
//                    shootThreeBallsOff();
//                    stopShooter();
//                    pathState = 3;
//                }
//                break;
//
//            case 3:
//
//
//        }
//    }
//
//}
