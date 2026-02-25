package org.firstinspires.ftc.teamcode.aold.leagemeet.autonomi;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.NrpOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Red Close 12")
@Configurable
public class RedClose12 extends NrpOpMode {

    private TelemetryManager panelsTelemetry;
    private Follower follower;
    private int pathState = 0;
    private Paths paths;

    private VoltageSensor batteryVoltageSensor;

    double currentVoltage = 0;


    double targetX = 132;
    double targetY = 132;

    // Optional: timers for delays
    private ElapsedTime pathTimer = new ElapsedTime();

    boolean pathStarted = false;

    @Override
    public void runOpMode() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(125.500, 118.700, Math.toRadians(36)));

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
        public PathChain P12_GoToIntake;
        public PathChain P2_IntakeOpen0300;
        public PathChain P3_ToTriShoot0603;
        public PathChain P4_Intake0603;
        public PathChain P5_ToTriShoot0906;
        public PathChain P52_ToIntake;
        public PathChain P6_Intakelast0906;
        public PathChain P7_Shootlast1209;

        public Paths(Follower follower) {
            P1_Shootfirst0303 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(125.500, 118.700),

                                    new Pose(98.800, 95.400)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0))

                    .build();

            P12_GoToIntake = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(98.800, 95.400),

                                    new Pose(101.209, 59.582)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            P2_IntakeOpen0300 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(101.209, 59.582),

                                    new Pose(126.200, 60.200)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            P3_ToTriShoot0603 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(126.200, 60.200),

                                    new Pose(85.300, 83.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            P4_Intake0603 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(85.300, 83.500),

                                    new Pose(123.100, 83.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            P5_ToTriShoot0906 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(123.100, 83.500),

                                    new Pose(83.600, 79.300)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            P52_ToIntake = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(83.600, 79.300),

                                    new Pose(91.967, 35.908)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            P6_Intakelast0906 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(91.967, 35.908),

                                    new Pose(125.000, 35.900)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            P7_Shootlast1209 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(125.000, 35.900),

                                    new Pose(91.000, 108.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(30))

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

        double graduatedDistance = Math.sqrt((targetX-turretX)*(targetX-turretX) + (targetY-turretY)*(targetY-turretY));
        // y= 0.000106346x^{4} - 0.0173809x^{3} + 0.666884x^{2} + 23.07246x + 1959.92984 = velocity
        double graduatedNearRPM = 0.000106346*Math.pow(graduatedDistance, 4)  - 0.0173809*Math.pow(graduatedDistance, 3) + 0.666884*Math.pow(graduatedDistance, 2) + 23.07246*graduatedDistance + 1959.92984;
//
//      // y = -0.000000214714x^{4} + 0.0000534017x^{3} - 0.00472863x^{2} + 0.172755x - 1.21858 = angle
        double graduatedNearAngle = -0.000000214714*Math.pow(graduatedDistance, 4) + 0.0000534017*Math.pow(graduatedDistance, 3) - 0.00472863*Math.pow(graduatedDistance, 2) + 0.172755*graduatedDistance + 1.21858;



        switch (pathState) {
            case 0:
                runShooterAtRPM(graduatedNearRPM, currentVoltage);
                if (!pathStarted) {
//                    runShooterAtRPM(graduatedNearRPM, currentVoltage);
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
                    if (pathTimer.milliseconds() >= 2500) {
                        TheJohnAustinHatch.setPosition(0.25);//open hatch
                        shootThreeBallsOn();
                    }
                    if (pathTimer.milliseconds() >= 3200) {
                        shootThreeBallsOff();
                        pathState = 2;
                    }
                }
                break;

            case 2:
                runShooterAtRPM(graduatedNearRPM, currentVoltage);
                if (!pathStarted) {
//                    runShooterAtRPM(graduatedNearRPM, currentVoltage);
                    follower.followPath(paths.P12_GoToIntake);
                    pathTimer.reset();
                    pathStarted = true;
                }

                if (!follower.isBusy()) {
                    pathStarted = false;
                    pathState = 3;
                }
                break;

            case 3:
                runShooterAtRPM(graduatedNearRPM, currentVoltage);
                if (!pathStarted) {
                    follower.followPath(paths.P2_IntakeOpen0300);
                    pathTimer.reset();
                    pathStarted = true;
                }
                if ((pathTimer.milliseconds() >= 400) && (pathTimer.milliseconds() < 1900)) {
                    Intake.setPower(-1);
                    Transfer.setPower(-1);
                }
                if (pathTimer.milliseconds() >= 1900) {
                    Transfer.setPower(0);
                }
                if ((!follower.isBusy()) && (pathTimer.milliseconds() >= 2300) ){
                    Intake.setPower(0);
                    pathStarted = false;
                    pathState = 4;
                }

                break;

            case 4:
                runShooterAtRPM(graduatedNearRPM, currentVoltage);
                if (!pathStarted) {
                    Intake.setPower(0);
                    follower.followPath(paths.P3_ToTriShoot0603);
                    pathTimer.reset();
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    Intake.setPower(0);
                    pathStarted = false;
                    pathState = 5;
                }
                break;

            case 5:
                runShooterAtRPM(graduatedNearRPM, currentVoltage);
                if (!follower.isBusy()) {
//                    runShooterAtRPM(graduatedNearRPM, currentVoltage);

                    if (pathTimer.milliseconds() >= 2500) {
                        TheJohnAustinHatch.setPosition(0.25);//open hatch
                        shootThreeBallsOn();
                    }
                    if (pathTimer.milliseconds() >= 3200) {
                        shootThreeBallsOff();
                        pathState = 6;
                    }
                }
                break;

            case 6:
                runShooterAtRPM(graduatedNearRPM, currentVoltage);
                if (!pathStarted) {
                    follower.followPath(paths.P4_Intake0603);
                    pathTimer.reset();
                    pathStarted = true;
                }
                if ((pathTimer.milliseconds() >= 400) && (pathTimer.milliseconds() < 1900)) {
                    Intake.setPower(-1);
                    Transfer.setPower(-1);
                }
                if (pathTimer.milliseconds() >= 1900) {
                    Transfer.setPower(0);
                }
                if ((!follower.isBusy()) && (pathTimer.milliseconds() >= 2300) ){
                    Intake.setPower(0);
                    pathStarted = false;
                    pathState = 7;
                }

                break;

            case 7:
                runShooterAtRPM(graduatedNearRPM, currentVoltage);
                if (!pathStarted) {
                    follower.followPath(paths.P5_ToTriShoot0906);
                    pathTimer.reset();
                    pathStarted = true;
                }
                if ((!follower.isBusy()) && (pathTimer.milliseconds() >= 1000) ){
                    pathStarted = false;
                    pathState = 8;
                }
                break;

            case 8:
                runShooterAtRPM(graduatedNearRPM, currentVoltage);
                if (!follower.isBusy()) {
//                    runShooterAtRPM(graduatedNearRPM, currentVoltage);

                    if (pathTimer.milliseconds() >= 2500) {
                        TheJohnAustinHatch.setPosition(0.25);//open hatch
                        shootThreeBallsOn();
                    }
                    if (pathTimer.milliseconds() >= 3200) {
                        shootThreeBallsOff();
                        pathState = 9;
                    }
                }
                break;

            case 9:
                runShooterAtRPM(graduatedNearRPM, currentVoltage);
                if (!pathStarted) {
//                    runShooterAtRPM(graduatedNearRPM, currentVoltage);
                    follower.followPath(paths.P52_ToIntake);
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
                    follower.followPath(paths.P6_Intakelast0906);
                    pathTimer.reset();
                    pathStarted = true;
                }
                if ((pathTimer.milliseconds() >= 400) && (pathTimer.milliseconds() < 1900)) {
                    Intake.setPower(-1);
                    Transfer.setPower(-1);
                }
                if (pathTimer.milliseconds() >= 1900) {
                    Transfer.setPower(0);
                }
                if ((!follower.isBusy()) && (pathTimer.milliseconds() >= 2300) ){
                    Intake.setPower(0);
                    pathStarted = false;
                    pathState = 11;
                }
                break;

            case 11:
                runShooterAtRPM(graduatedNearRPM, currentVoltage);
                if (!pathStarted) {
                    follower.followPath(paths.P7_Shootlast1209);
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
                if (!follower.isBusy()) {
//                    runShooterAtRPM(graduatedNearRPM, currentVoltage);
                    if ((pathTimer.milliseconds() >= 2500) && (pathTimer.milliseconds() < 4450)) {
                        TheJohnAustinHatch.setPosition(0.25);//open hatch
                        shootThreeBallsOn();
                    }
                    if (pathTimer.milliseconds() >= 4450) {
                        shootThreeBallsOff();
                        pathState = 13;
                    }
                }

            case 13:
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