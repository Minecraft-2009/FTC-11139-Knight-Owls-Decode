package org.firstinspires.ftc.teamcode.aold.leagemeet.autonomi;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.NrpOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Red Close 18")
@Configurable
public class RedClose18 extends NrpOpMode {

    private TelemetryManager panelsTelemetry;
    private Follower follower;
    private int pathState = 0;
    private Paths paths;

    private VoltageSensor batteryVoltageSensor;

    double currentVoltage = 0;

    double targetX = 135;
    double targetY = 132;

    // Optional: timers for delays
    private ElapsedTime pathTimer = new ElapsedTime();

    boolean pathStarted = false;

    @Override
    public void runOpMode() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(125.500, 118.700, Math.toRadians(36)));// 128.57, 119.8799

        paths = new Paths(follower);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();



        panelsTelemetry.debug("Status", "Initialized - Waiting for start");
        panelsTelemetry.update(telemetry);

        initControls();

        waitForStart();
        pathTimer.reset();

        panelsTelemetry.debug("Status", "Auto Started");
        panelsTelemetry.update(telemetry);


        // Main linear loop
        while (opModeIsActive()) {
            follower.update();  // necessary


            // Turret AutoLock
            Pose currentPose = follower.getPose();


            double robotX = follower.getPose().getX();
            double robotY = follower.getPose().getY();
            double robotHeadingRad = currentPose.getHeading();
            robotHeadingRad = (robotHeadingRad + 2*Math.PI) % (2*Math.PI);
            double robotHeadingDeg = robotHeadingRad * (180/Math.PI);
            double turretX = robotX - (2.5 * Math.cos(robotHeadingRad) );
            double turretY = robotY - (2.5 * Math.sin(robotHeadingRad) );
            double moddedRobotHeadingRad = robotHeadingRad/(2*Math.PI)-0.25;
            double pos = 0.75 - (1/(2*Math.PI))*Math.atan((-1*turretY+132)/(-1*turretX+132)) + moddedRobotHeadingRad;
            if (pos > 1){
                pos --;
            }
            pos = (pos-0.5)*360.0/320.0+0.5;
            if (pos > 1){
                pos = 1;
            }
            pos = Math.max(0.0, Math.min(1.0, pos));

            TurretL.setPosition(pos);
            TurretR.setPosition(pos);

            currentVoltage = batteryVoltageSensor.getVoltage();


            double graduatedDistance = Math.sqrt((targetX-turretX)*(targetX-turretX) + (targetY-turretY)*(targetY-turretY)); // distance

            // y=0.000125806x^{4} - 0.0259667x^{3} + 1.70772x^{2} - 32.29572x + 3077.99517
            double graduatedNearRPM = 0.000125806*Math.pow(graduatedDistance, 4) - 0.0259667*Math.pow(graduatedDistance, 3) + 1.70772*Math.pow(graduatedDistance, 2) - 32.29572*graduatedDistance + 3077.99517;

//                //y= 0.0000000509332x^{4} - 0.0000184529x^{3} + 0.00241167x^{2} - 0.13161x + 3.29491
            double graduatedNearAngle = 0.0000000509332*Math.pow(graduatedDistance, 4) - 0.0000184529*Math.pow(graduatedDistance, 3) + 0.00241167*Math.pow(graduatedDistance, 2) - 0.13161*graduatedDistance + 3.29491;

            graduatedNearAngle = Math.min(1.0, Math.max(0.45, graduatedNearAngle));

            graduatedNearRPM = Math.min(3550, Math.max(3000, graduatedNearRPM));

            if (graduatedDistance <= 37) {
                graduatedNearAngle = 1;
            }

            Angler.setPosition(graduatedNearAngle);


            autonomousPathUpdate();



            // Telemetry (unchanged)
            panelsTelemetry.debug("Path State", pathState);
            panelsTelemetry.debug("X", String.format("%.2f", follower.getPose().getX()));
            panelsTelemetry.debug("Y", String.format("%.2f", follower.getPose().getY()));
            panelsTelemetry.debug("Heading °", String.format("%.1f", Math.toDegrees(follower.getPose().getHeading())));
            panelsTelemetry.update(telemetry);

            telemetry.update();
        }
    }

    public static class Paths {
        public PathChain P1_Shootfirst0303;
        public PathChain P1_2_Intake0300;
        public PathChain P2_ToTriShoot0606;
        public PathChain P3_OpenGateIntake0600;
        public PathChain P4_ToTriShoot0903;
        public PathChain P5_OpenGateIntake0900;
        public PathChain P6_ToTriShoot1203;
        public PathChain P7_OpenGateIntake1200;
        public PathChain P8_ToTriShoot1203;
        public PathChain P9_Intake1strow1203;
        public PathChain P10_Shootlastballs1806;
            //todo
        double gateX = 123.3699;// 123.7417 //start=124.902 => 1.1603
        double gateY = 60.2781; // 62.2669 //start=120.6888 => 58.4219
        double gateAngle = 21.948; //18.68 //start=32.732 => 14.052

        public Paths(Follower follower) {
            P1_Shootfirst0303 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(125.500, 118.700),

                                    new Pose(98.800, 95.400)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0))

                    .build();

            P1_2_Intake0300 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(98.800, 95.400),
                                    new Pose(97.508, 55.651),
                                    new Pose(91.729, 59.604),
                                    new Pose(126.200, 59.400)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            P2_ToTriShoot0606 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(126.200, 59.400),
                                    new Pose(104.028, 70.963),
                                    new Pose(86.300, 87.100)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(20))

                    .build();

            P3_OpenGateIntake0600 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(86.300, 87.100),
                                    new Pose(gateX, gateY)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(20), Math.toRadians(gateAngle))

                    .build();

            P4_ToTriShoot0903 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(gateX, gateY),
                                    new Pose(104.036, 69.129),
                                    new Pose(86.200, 83.100)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(gateAngle), Math.toRadians(20))

                    .build();

            P5_OpenGateIntake0900 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(86.200, 83.100),
                                    new Pose(gateX, gateY)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(20), Math.toRadians(gateAngle))

                    .build();

            P6_ToTriShoot1203 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(gateX, gateY),
                                    new Pose(102.144, 67.673),
                                    new Pose(86.300, 82.900)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(gateAngle), Math.toRadians(20))

                    .build();

            P7_OpenGateIntake1200 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(86.300, 82.900),
                                    new Pose(gateX, gateY)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(20), Math.toRadians(gateAngle))

                    .build();

            P8_ToTriShoot1203 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(gateX, gateY),

                                    new Pose(86.300, 82.900)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(gateAngle), Math.toRadians(20))

                    .build();

            P9_Intake1strow1203 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(86.300, 82.900),
                                    new Pose(106.982, 79.407),
                                    new Pose(125.900, 83.600)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(20), Math.toRadians(0))

                    .build();

            P10_Shootlastballs1806 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(125.900, 83.600),

                                    new Pose(91.000, 117.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(35))

                    .build();
        }
    }

//    public static double nearRPM(double distance) {
//        return 0.000106346*Math.pow(distance, 4)  - 0.0173809*Math.pow(distance, 3) + 0.666884*Math.pow(distance, 2) + 23.07246*distance + 1959.92984;
//    }


    public void autonomousPathUpdate() {

        // Graduated information
        Pose currentPose = follower.getPose();
        double robotX = follower.getPose().getX();
        double robotY = follower.getPose().getY();
        double robotHeadingRad = currentPose.getHeading();
        robotHeadingRad = (robotHeadingRad + 2*Math.PI) % (2*Math.PI);
        double robotHeadingDeg = robotHeadingRad * (180/Math.PI);
        double turretX = robotX - (2.5 * Math.cos(robotHeadingRad) );
        double turretY = robotY - (2.5 * Math.sin(robotHeadingRad) );

        double graduatedDistance = Math.sqrt((targetX-turretX)*(targetX-turretX) + (targetY-turretY)*(targetY-turretY)); // distance

        // y=0.000125806x^{4} - 0.0259667x^{3} + 1.70772x^{2} - 32.29572x + 3077.99517
        double graduatedNearRPM = 0.000125806*Math.pow(graduatedDistance, 4) - 0.0259667*Math.pow(graduatedDistance, 3) + 1.70772*Math.pow(graduatedDistance, 2) - 32.29572*graduatedDistance + 3077.99517;

//                //y= 0.0000000509332x^{4} - 0.0000184529x^{3} + 0.00241167x^{2} - 0.13161x + 3.29491
        double graduatedNearAngle = 0.0000000509332*Math.pow(graduatedDistance, 4) - 0.0000184529*Math.pow(graduatedDistance, 3) + 0.00241167*Math.pow(graduatedDistance, 2) - 0.13161*graduatedDistance + 3.29491;

        graduatedNearAngle = Math.min(1.0, Math.max(0.45, graduatedNearAngle));

        graduatedNearRPM = Math.min(3550, Math.max(3000, graduatedNearRPM));

        if (graduatedDistance <= 37) {
            graduatedNearAngle = 1;
        }

        Angler.setPosition(graduatedNearAngle);


        switch (pathState) {
            case 0:
                runShooterAtRPM(graduatedNearRPM, currentVoltage);
                if (!pathStarted) {
                    follower.followPath(paths.P1_Shootfirst0303);
                    pathTimer.reset();
                    pathStarted = true;
                }

                if (!follower.isBusy()) {
                    pathStarted = false;
                    pathState = 1;
                }
            break;

            case 1:
                runShooterAtRPM(graduatedNearRPM, currentVoltage);
                if (!follower.isBusy()) {
//                    runShooterAtRPM(graduatedNearRPM, currentVoltage);
                    if ((pathTimer.milliseconds() >= 1600) && (pathTimer.milliseconds() < 2350)) {
                        shootThreeBallsOn();
                    }
                    if (pathTimer.milliseconds() >= 2350) {
                        shootThreeBallsOff();
                        pathState = 2;
                    }
                }
                break;

            case 2:
                runShooterAtRPM(graduatedNearRPM, currentVoltage);
                if (!pathStarted) {
                    follower.followPath(paths.P1_2_Intake0300);
                    pathTimer.reset();
                    pathStarted = true;
                }
                if ((pathTimer.milliseconds() >= 300) && (pathTimer.milliseconds() < 1900)) {
                    Intake.setPower(-1);
                    Transfer.setPower(-1);
                }
                if (pathTimer.milliseconds() >= 1900) {
                    Transfer.setPower(0);
                }
                if ((!follower.isBusy()) && (pathTimer.milliseconds() >= 2200) ){
                    Intake.setPower(0);
                    pathStarted = false;
                    pathState = 3;
                }
                break;

            case 3:
                runShooterAtRPM(graduatedNearRPM, currentVoltage);
                if (!pathStarted) {
                    follower.followPath(paths.P2_ToTriShoot0606);
//                    runShooterAtRPM(graduatedNearRPM, currentVoltage);
                    pathTimer.reset();
                    pathStarted = true;
                }
                if ((!follower.isBusy()) && (pathTimer.milliseconds() >= 100) ){
                    pathStarted = false;
                    pathState = 4;
                }

                break;

            case 4:
                runShooterAtRPM(graduatedNearRPM, currentVoltage);
                if (!pathStarted) {
                    pathTimer.reset();
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
//                    runShooterAtRPM(graduatedNearRPM, currentVoltage);
                    if (pathTimer.milliseconds() >= 700) {
                        shootThreeBallsOn();
                    }
                    if (pathTimer.milliseconds() >= 1450) {
                        shootThreeBallsOff();
                        pathStarted = false;
                        pathState = 5;
                    }
                }
                break;

            case 5:
                runShooterAtRPM(graduatedNearRPM, currentVoltage);
                if (!pathStarted) {
                    Intake.setPower(0);
                    follower.followPath(paths.P3_OpenGateIntake0600);
                    pathTimer.reset();
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    pathState = 6;
                }
                break;

            case 6:
                runShooterAtRPM(graduatedNearRPM, currentVoltage);
                if (!pathStarted) {
                    pathTimer.reset();
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    if ((pathTimer.milliseconds() >= 0) && (pathTimer.milliseconds() < 1700)) {
                        TheJohnAustinHatch.setPosition(0);
                        Intake.setPower(-1);
                        Transfer.setPower(-1);
                    }
                    if ((pathTimer.milliseconds() >= 1700) && (pathTimer.milliseconds() < 2200)) {
                        Transfer.setPower(0);
                    }
                    if (pathTimer.milliseconds() >= 2200) {
                        Intake.setPower(0);
                        Transfer.setPower(0);
                        pathStarted = false;
                        pathState = 7;
                    }
                }
                break;

            case 7:
                runShooterAtRPM(graduatedNearRPM, currentVoltage);
                if (!pathStarted) {
                    follower.followPath(paths.P4_ToTriShoot0903);
                    pathTimer.reset();
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    pathState = 8;
                }
                break;

            case 8:
                runShooterAtRPM(graduatedNearRPM, currentVoltage);
                if (!pathStarted) {
                    pathTimer.reset();
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
//                    runShooterAtRPM(graduatedNearRPM, currentVoltage);
                    if (pathTimer.milliseconds() >= 700) {
                        shootThreeBallsOn();
                    }
                    if (pathTimer.milliseconds() >= 1600) {
                        shootThreeBallsOff();
                        pathStarted = false;
                        pathState = 9;
                    }
                }
                break;

            case 9:
                runShooterAtRPM(graduatedNearRPM, currentVoltage);
                if (!pathStarted) {
                    Intake.setPower(0);
                    follower.followPath(paths.P5_OpenGateIntake0900);
                    pathTimer.reset();
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    pathState = 10;
                }
                break;

            case 10:
                runShooterAtRPM(graduatedNearRPM, currentVoltage);
                if (!pathStarted) {
                    pathTimer.reset();
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    if ((pathTimer.milliseconds() >= 0) && (pathTimer.milliseconds() < 1700)) {
                        TheJohnAustinHatch.setPosition(0);
                        Intake.setPower(-1);
                        Transfer.setPower(-1);
                    }
                    if ((pathTimer.milliseconds() >= 1700) && (pathTimer.milliseconds() < 2200)) {
                        Transfer.setPower(0);
                    }
                    if (pathTimer.milliseconds() >= 2200) {
                        Intake.setPower(0);
                        Transfer.setPower(0);
                        pathStarted = false;
                        pathState = 11;
                    }
                }
                break;
//
            case 11:
                runShooterAtRPM(graduatedNearRPM, currentVoltage);
                if (!pathStarted) {
                    follower.followPath(paths.P6_ToTriShoot1203);
                    pathTimer.reset();
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    pathState = 12;
                }
                break;

            case 12:
                runShooterAtRPM(graduatedNearRPM, currentVoltage);
                if (!pathStarted) {
                    pathTimer.reset();
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
//                    runShooterAtRPM(graduatedNearRPM, currentVoltage);
                    if (pathTimer.milliseconds() >= 700) {
                        shootThreeBallsOn();
                    }
                    if (pathTimer.milliseconds() >= 1600) {
                        shootThreeBallsOff();
                        pathStarted = false;
                        pathState = 13;
                    }
                }
                break;

            case 13:
                runShooterAtRPM(graduatedNearRPM, currentVoltage);
                if (!pathStarted) {
                    Intake.setPower(0);
                    follower.followPath(paths.P7_OpenGateIntake1200);
                    pathTimer.reset();
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    pathState = 14;
                }
                break;

            case 14:
                if (!pathStarted) {
                    pathTimer.reset();
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    if ((pathTimer.milliseconds() >= 0) && (pathTimer.milliseconds() < 1700)) {
                        TheJohnAustinHatch.setPosition(0);
                        Intake.setPower(-1);
                        Transfer.setPower(-1);
                    }
                    if ((pathTimer.milliseconds() >= 1700) && (pathTimer.milliseconds() < 2200)) {
                        Transfer.setPower(0);
                    }
                    if (pathTimer.milliseconds() >= 2200) {
                        Intake.setPower(0);
                        Transfer.setPower(0);
                        pathStarted = false;
                        pathState = 15;
                    }
                }
                break;

            case 15:
                runShooterAtRPM(graduatedNearRPM, currentVoltage);
                if (!pathStarted) {
                    follower.followPath(paths.P8_ToTriShoot1203);
                    pathTimer.reset();
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    pathState = 16;
                }
                break;

            case 16:
                runShooterAtRPM(graduatedNearRPM, currentVoltage);
                if (!pathStarted) {
                    pathTimer.reset();
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
//                    runShooterAtRPM(graduatedNearRPM, currentVoltage);
                    if (pathTimer.milliseconds() >= 700) {
                        shootThreeBallsOn();
                    }
                    if (pathTimer.milliseconds() >= 1600) {
                        shootThreeBallsOff();
                        pathStarted = false;
                        pathState = 17;
                    }
                }
                break;
            case 17:
                runShooterAtRPM(graduatedNearRPM, currentVoltage);
                if (!pathStarted) {
                    follower.followPath(paths.P9_Intake1strow1203);
                    pathTimer.reset();
                    pathStarted = true;
                }
                if ((pathTimer.milliseconds() >= 1000) && (pathTimer.milliseconds() < 2300)) {
                    Intake.setPower(-1);
                    Transfer.setPower(-1);
                }
                if (pathTimer.milliseconds() >= 2200) {
                    Transfer.setPower(0);
                }
                if ((!follower.isBusy()) && (pathTimer.milliseconds() >= 3200) ){
                    Intake.setPower(0);
                    pathStarted = false;
                    pathState = 18;
                }
                break;
//
            case 18:
                runShooterAtRPM(graduatedNearRPM, currentVoltage);
                if (!pathStarted) {
                    follower.followPath(paths.P10_Shootlastballs1806);
                    pathTimer.reset();
                    pathStarted = true;
                }
//                if (pathTimer.milliseconds() >= 1200) {
////                    runShooterAtRPM(graduatedNearRPM, currentVoltage);
//                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    pathState = 19;
                }
                break;
//
            case 19:
                runShooterAtRPM(graduatedNearRPM, currentVoltage);
                if (!follower.isBusy()) {
//                    runShooterAtRPM(graduatedNearRPM, currentVoltage);

                    if (pathTimer.milliseconds() >= 2000) {
                        shootThreeBallsOn();
                    }
                    if (pathTimer.milliseconds() >= 2900) {
                        shootThreeBallsOff();
                        pathState = 20;
                    }
                }
                break;


            case 20:
                if (!follower.isBusy()) {
                    follower.breakFollowing();
                    pathState = 99;
                }
                break;

            case 99:
                requestOpModeStop();
                break;

        }
    }

}