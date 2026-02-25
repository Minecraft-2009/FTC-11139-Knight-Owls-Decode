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
//@Autonomous(name = "Blue Far 3 - New")
//@Configurable
//public class BlueFar extends NrpOpMode {
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
//        follower.setStartingPose(new Pose(54.200, 9.000, Math.toRadians(90))); // Blue close start
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
//            // double moddedRobotHeadingRad = robotHeadingRad/(2*Math.PI)-0.25;
//            // double pos = 0.75 - (1/(2*Math.PI))*Math.atan((-1*turretY+132)/(-1*turretX+132)) + moddedRobotHeadingRad;
//            // if (pos > 1){
//            //     pos --;
//            // }
//            // pos = (pos-0.5)*360.0/320.0+0.5;
//            // if (pos > 1){
//            //     pos = 1;
//            // }
//            // pos = Math.max(0.0, Math.min(1.0, pos));
//
//            TurretL.setPosition(0.5); //TODO: No autolock! Must be manually oriented
//            TurretR.setPosition(0.5);
//
//            // Dynamic Gradual Hood Angle
//            double graduatedDistance = Math.sqrt((targetX-turretX)*(targetX-turretX) + (targetY-turretY)*(targetY-turretY)); // distance
//            // double graduatedNearRPM = 0.000106346*Math.pow(graduatedDistance, 4)  - 0.0173809*Math.pow(graduatedDistance, 3) + 0.666884*Math.pow(graduatedDistance, 2) + 23.07246*graduatedDistance + 1959.92984;
//            // double graduatedNearAngle = -0.000000214714*Math.pow(graduatedDistance, 4) + 0.0000534017*Math.pow(graduatedDistance, 3) - 0.00472863*Math.pow(graduatedDistance, 2) + 0.172755*graduatedDistance + 1.21858;
//            // if (graduatedNearAngle < 0.45) {
//            //     graduatedNearAngle = 0.45;
//            // }
//            Angler.setPosition(0.55);
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
//
//    public static class Paths {
//        public PathChain P1_Moveforwardshoot0303;
//        public PathChain P2_park0303;
//
//        public Paths(Follower follower) {
//            P1_Moveforwardshoot0303 = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(54.200, 9.000),
//
//                                    new Pose(54.200, 12.100)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
//
//                    .build();
//
//            P2_park0303 = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(54.200, 12.100),
//
//                                    new Pose(54.200, 34.000)
//                            )
//                    ).setConstantHeadingInterpolation(Math.toRadians(90))
//
//                    .build();
//        }
//    }
//
//
//    //    public static double nearRPM(double distance) {
//    //        return 0.000106346*Math.pow(distance, 4)  - 0.0173809*Math.pow(distance, 3) + 0.666884*Math.pow(distance, 2) + 23.07246*distance + 1959.92984;
//    //    }
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
//        //
//        //      // y = -0.000000214714x^{4} + 0.0000534017x^{3} - 0.00472863x^{2} + 0.172755x - 1.21858 = angle
//        double graduatedNearAngle = -0.000000214714*Math.pow(graduatedDistance, 4) + 0.0000534017*Math.pow(graduatedDistance, 3) - 0.00472863*Math.pow(graduatedDistance, 2) + 0.172755*graduatedDistance + 1.21858;
//
//
//        switch (pathState) {
//            case 0:
//                if (!pathStarted) {
//                    runShooterAtRPM(5000);
//                    follower.followPath(paths.P1_Moveforwardshoot0303);
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
//                    runShooterAtRPM(4900);
//
//                    if ((pathTimer.milliseconds() >= 4900) && (pathTimer.milliseconds() < 5300)) {
//                        TheJohnAustinHatch.setPosition(0.25); //TODO: IS THIS THE RIGHT POSITION???
//                        Intake.setPower(-0.7);
//                        Transfer.setPower(-0.8);
//                    }
//                    if ((pathTimer.milliseconds() >= 5300) && (pathTimer.milliseconds() < 5350)) {
//                        TheJohnAustinHatch.setPosition(0.25); //TODO: IS THIS THE RIGHT POSITION???
//                        Intake.setPower(0);
//                        Transfer.setPower(0);
//                    }
//                    if ((pathTimer.milliseconds() >= 5350) && (pathTimer.milliseconds() < 5600)) {
//                        TheJohnAustinHatch.setPosition(0.25); //TODO: IS THIS THE RIGHT POSITION???
//                        Intake.setPower(-0.7);
//                        Transfer.setPower(-0.8);
//                    }
//                    if ((pathTimer.milliseconds() >= 5600) && (pathTimer.milliseconds() < 5650)) {
//                        TheJohnAustinHatch.setPosition(0.25); //TODO: IS THIS THE RIGHT POSITION???
//                        Intake.setPower(0);
//                        Transfer.setPower(0);
//                    }
//                    if ((pathTimer.milliseconds() >= 5650) && (pathTimer.milliseconds() < 5900)) {
//                        TheJohnAustinHatch.setPosition(0.25); //TODO: IS THIS THE RIGHT POSITION???
//                        Intake.setPower(-0.7);
//                        Transfer.setPower(-0.8);
//                    }
//                    if ((pathTimer.milliseconds() >= 5900) && (pathTimer.milliseconds() < 5950)) {
//                        TheJohnAustinHatch.setPosition(0.25); //TODO: IS THIS THE RIGHT POSITION???
//                        Intake.setPower(0);
//                        Transfer.setPower(0);
//                    }
//                    if ((pathTimer.milliseconds() >= 5950) && (pathTimer.milliseconds() < 6200)) {
//                        TheJohnAustinHatch.setPosition(0.25); //TODO: IS THIS THE RIGHT POSITION???
//                        Intake.setPower(-0.7);
//                        Transfer.setPower(-0.8);
//                    }
//                    if ((pathTimer.milliseconds() >= 6200) && (pathTimer.milliseconds() < 6250)) {
//                        TheJohnAustinHatch.setPosition(0.25); //TODO: IS THIS THE RIGHT POSITION???
//                        Intake.setPower(0);
//                        Transfer.setPower(0);
//                    }
//                    if ((pathTimer.milliseconds() >= 6250) && (pathTimer.milliseconds() < 6500)) {
//                        TheJohnAustinHatch.setPosition(0.25); //TODO: IS THIS THE RIGHT POSITION???
//                        Intake.setPower(-0.7);
//                        Transfer.setPower(-0.8);
//                    }
//                    if ((pathTimer.milliseconds() >= 6500) && (pathTimer.milliseconds() < 6550)) {
//                        TheJohnAustinHatch.setPosition(0.25); //TODO: IS THIS THE RIGHT POSITION???
//                        Intake.setPower(0);
//                        Transfer.setPower(0);
//                    }
//                    if ((pathTimer.milliseconds() >= 6550) && (pathTimer.milliseconds() < 6800)) {
//                        TheJohnAustinHatch.setPosition(0.25); //TODO: IS THIS THE RIGHT POSITION???
//                        Intake.setPower(-0.7);
//                        Transfer.setPower(-0.8);
//                    }
//                    if ((pathTimer.milliseconds() >= 6800) && (pathTimer.milliseconds() < 6850)) {
//                        TheJohnAustinHatch.setPosition(0.25); //TODO: IS THIS THE RIGHT POSITION???
//                        Intake.setPower(0);
//                        Transfer.setPower(0);
//                    }
//                    if ((pathTimer.milliseconds() >= 6850) && (pathTimer.milliseconds() < 7200)) {
//                        TheJohnAustinHatch.setPosition(0.25); //TODO: IS THIS THE RIGHT POSITION???
//                        Intake.setPower(-0.7);
//                        Transfer.setPower(-0.8);
//                    }
//                    if (pathTimer.milliseconds() >= 7200) {
//                        TheJohnAustinHatch.setPosition(0.25); //TODO: IS THIS THE RIGHT POSITION???
//                        Intake.setPower(0);
//                        Transfer.setPower(0);
//                    }
//                    if (pathTimer.milliseconds() >= 7500) {
//                        stopShooter();
//                        pathStarted = false;
//                        pathState = 2;
//                    }
//                }
//                break;
//
//            case 2:
//                if ((!pathStarted) && (pathTimer.milliseconds() >= 10000)) {
//                    follower.followPath(paths.P2_park0303);
//                    pathTimer.reset();
//                    pathStarted = true;
//                }
//                if ((!follower.isBusy()) && (pathTimer.milliseconds() >= 12000)){
//                pathStarted = false;
//                pathState = 3;
//                }
//
//            break;
//
//            case 3:
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