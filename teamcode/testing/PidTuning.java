package org.firstinspires.ftc.teamcode.testing;


//
//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.telemetry.TelemetryManager;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.math.MathFunctions;
//import com.pedropathing.math.Vector;
//import com.bylazar.configurables.PanelsConfigurables;
//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.configurables.annotations.IgnoreConfigurable;
//import com.bylazar.field.FieldManager;
//import com.bylazar.field.PanelsField;
//import com.bylazar.field.Style;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

//


import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.TelemetryManager;
//import com.pedropathing.control.PIDFController;
//import com.pedropathing.control.PIDFCoefficients;
//import com.pedropathing.control.PIDFController;

import org.firstinspires.ftc.teamcode.PIDFControllerNRP;

import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.VoltageSensor;


import org.firstinspires.ftc.teamcode.NrpOpMode;
//import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;



import org.firstinspires.ftc.teamcode.PIDFControllerNRP.*;

import java.util.ArrayList;


@TeleOp(name = "PIDF Tuning", group = "ATest")

public class PidTuning extends NrpOpMode {

    private VoltageSensor batteryVoltageSensor;


    //
    public static double LcurrentRPM = 0;
    public static double RcurrentRPM = 0;

    PIDFControllerNRP controller = new PIDFControllerNRP();
    FtcDashboard dashboard = FtcDashboard.getInstance();



    public static Follower follower;

    private boolean launcherOn;


    double regularDivBy = 1;
    double curVelocity ;
    double targetPower;





    double tunedVoltage = 12.15;




    public void runOpMode() {

        //init stuff
        initControls();
//        initVoltageSensor();
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        try {
            initCommands();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        //pedro turret stuff
        follower = Constants.createFollower(hardwareMap);
        //start with end of auto
        follower.setStartingPose(new Pose(72.000, 72.000, Math.toRadians(90)));
        follower.update();


        waitForStart();

        boolean isGraduated = true;

//        double TestRPM = 1500;


        double testRPM = 4650; //TODO
        double hoodAngle = 0.8;

        double testAngle = 0.450;
        double currentVoltage = 1;


        if (opModeIsActive()) {

            // Wheel code
            while (opModeIsActive()) {

                if (gamepad2.left_stick_y != 0 || gamepad2.left_stick_x != 0 || gamepad2.right_stick_x != 0) {
                    double r = Math.hypot(gamepad2.left_stick_x, gamepad2.left_stick_y);
                    double robotAngle = Math.atan2(gamepad2.left_stick_y, gamepad2.left_stick_x) - Math.PI / 4;
                    double turn = gamepad2.right_stick_x;
                    double rightX = turn * turn * turn;
                    final double v1 = (-r * Math.cos(robotAngle) + rightX);
                    final double v2 = (-r * Math.sin(robotAngle) - rightX);
                    final double v3 = (-r * Math.sin(robotAngle) + rightX);
                    final double v4 = (-r * Math.cos(robotAngle) - rightX);
                    LeftBack.setPower(v1 * regularDivBy);
                    RightBack.setPower(v2 * regularDivBy);
                    LeftFront.setPower(v3 * regularDivBy);
                    RightFront.setPower(v4 * regularDivBy);


//                    telemetry.addData("LB", v1 * regularDivBy);
//                    telemetry.addData("RB", v2 * regularDivBy);
//                    telemetry.addData("LF", v3 * regularDivBy);
//                    telemetry.addData("RF", v4 * regularDivBy);
//                    telemetry.update();
                } else {
                    LeftFront.setPower(0);
                    LeftBack.setPower(0);
                    RightFront.setPower(0);
                    RightBack.setPower(0);
                }
                
                

//                if (batteryVoltageSensor != null) {
                currentVoltage = batteryVoltageSensor.getVoltage();
                    // Use the voltage value
//                } else {
//                    // Handle the error, maybe log a telemetry message
//                    telemetry.addData("Error", "Voltage Sensor not found/initialized");
//                }



                /*
                Gamepad 1:
                dpad up/down/left: Angler

                Gamepad 2:
                dpad up/down: PID shooter on/off
                a/b: John Auston Hatch
                triggers: intake (in/out)
                bumpers: transfer (in/out)
                x: Turret @ 0.5

                 */


//                 In memento Ioanni Austinis Hatchionis
                if (gamepad2.a) {
                    TheJohnAustinHatch.setPosition(0);//hatch close
                } else if (gamepad2.b) {
                    TheJohnAustinHatch.setPosition(0.25);//hatch open
                }

                // Intake
                if (gamepad2.right_trigger >= 0.2) {
                    if (gamepad2.right_trigger >= 0.2) {
                        Intake.setPower(-1);
                    } else {
                        Intake.setPower(0);
                    }
                } else {
                    if (gamepad2.left_trigger >= 0.2) {
                        Intake.setPower(0.5);
                    } else {
                        Intake.setPower(0);
                    }
                }

                //Transfer
                if (gamepad2.right_bumper) {
                    Transfer.setPower(-1);
                } else if (gamepad2.left_bumper) {
                    Transfer.setPower(0.4);
                } else {
                    Transfer.setPower(0);
                }


                // location reset
                if (gamepad1.x) {
                    follower.setPose(new Pose(8.500, 10.00, Math.toRadians(180)));
                    follower.update();
                }


                follower.update();
                Pose pose = follower.getPose();


                double targetX = 132;
                double targetY = 132;

                double robotX = follower.getPose().getX();
                double robotY = follower.getPose().getY();

                double robotHeadingRad = pose.getHeading();
                robotHeadingRad = (robotHeadingRad + 2 * Math.PI) % (2 * Math.PI);
                double robotHeadingDeg = robotHeadingRad * (180 / Math.PI);

                double turretX = robotX - (2.5 * Math.cos(robotHeadingRad));
                double turretY = robotY - (2.5 * Math.sin(robotHeadingRad));

//
                double moddedRobotHeadingRad = robotHeadingRad / (2 * Math.PI) - 0.25;
                double pos = 0.75 - (1 / (2 * Math.PI)) * Math.atan((-1 * turretY + 132) / (-1 * turretX + 132)) + moddedRobotHeadingRad;
                if (pos > 1) {
                    pos--;
                }
                pos = (pos - 0.5) * 360.0 / 320.0 + 0.5;
                if (pos > 1) {
                    pos = 1;
                }

                pos = Math.max(0.0, Math.min(1.0, pos));


                TurretL.setPosition(pos);
                TurretR.setPosition(pos);

                double graduatedDistance = Math.sqrt((targetX - turretX) * (targetX - turretX) + (targetY - turretY) * (targetY - turretY)); // distance


//                 In memento Ioanni Austinis Hatchionis
                if (gamepad2.a) {
                    TheJohnAustinHatch.setPosition(0);//hatch close
                } else if (gamepad2.b) {
                    TheJohnAustinHatch.setPosition(0.25);//hatch open
                }


                // Intake
                if (gamepad2.right_trigger >= 0.2) {
                    if (gamepad2.right_trigger >= 0.2) {
                        Intake.setPower(-1);
                    } else {
                        Intake.setPower(0);
                    }
                } else {
                    if (gamepad2.left_trigger >= 0.2) {
                        Intake.setPower(0.5);
                    } else {
                        Intake.setPower(0);
                    }
                }

                //Transfer
                if (gamepad2.right_bumper) {
                    Transfer.setPower(-1);
                } else if (gamepad2.left_bumper) {
                    Transfer.setPower(0.4);
                } else {
                    Transfer.setPower(0);
                }


//                // Testing shooter motor
//
//                if (gamepad2.dpad_left) {
//                    testRPM = 3600;
//                    runShooterAtRPM(testRPM);
//                    RGBlight.setPosition(0.279);
//                } else if (gamepad2.dpad_up) {
//                    testRPM = 3900;
//                    RGBlight.setPosition(0.4);
//                    runShooterAtRPM(testRPM);
//                } else if (gamepad2.dpad_right) {
//                    testRPM = 4450;
//                    RGBlight.setPosition(0.611);
//                    runShooterAtRPM(testRPM);
//
//                } else if (gamepad2.xWasPressed()) {
//                    testRPM = 4950;
//                    RGBlight.setPosition(0.7);
//                    runShooterAtRPM(testRPM);
//                }
//
////                else if (gamepad2.yWasPressed()) {
////                    testRPM = 4000;
////                    RGBlight.setPosition(0.55);
////                    runShooterAtRPM(testRPM);
////                }
//                else if (gamepad2.dpad_down) {
//                    stopShooter();
//                    RGBlight.setPosition(0.2);

                /* //TODO Color tuning
                0.0-0.277 = Off
                0.278 = Red
                0.33 = Orange
                0.388 = Yellow
                0.444 = Sage
                0.5 = Green
                0.555 = Azure
                0.611 = Blue
                0.666 = Indigo
                0.722 = Violet
                0.723-1 = White

                 */
//            testRPM = 3300;
            //TODO: Test Graduated PIDF
            if (gamepad2.dpad_up) {
                launcherOn = true;
            }
            if (gamepad2.dpad_left) {
                testRPM -= 50;
                launcherOn = true;
            }
            if (gamepad2.dpad_right) {
                testRPM += 50;
                launcherOn = true;
            }
            if (gamepad2.dpad_down) {
                launcherOn = false;
            }
            if (gamepad2.xWasPressed()) {
                hoodAngle -= 0.0125;
            }
            if (gamepad2.yWasPressed()) {
                hoodAngle += 0.0125;
            }
//



            double RcurrentTPS = RightShooter.getVelocity();
            RcurrentRPM = RcurrentTPS * 60 / TICKS_PER_REV;
//

            double LcurrentTPS = LeftShooter.getVelocity();
            LcurrentRPM = LcurrentTPS * 60 / TICKS_PER_REV;
//
            double currentRPM = (RcurrentRPM + LcurrentRPM) / 2;
//            RGBlight.setPosition(0.29 + (LcurrentRPM - 2000) * 0.41 / 3000);
            RGBlight.setPosition(0.5 - (testRPM - RcurrentRPM) / 2000);

            P = 0.002;
            I = 0.00;
            kS = 0.0004;
            kV = 0.00041975;
            double testTPS = testRPM* TICKS_PER_REV / 60.0;
            controller.setPIDF(P, I, 0.0, kV * testTPS + kS);
            curVelocity = LeftShooter.getVelocity();
            double error = testTPS-curVelocity;

            double feedforward = kS + kV*testTPS;
            double feedback = error*P;
            double batteryCompensation = tunedVoltage/currentVoltage;
            if (launcherOn) {
//              ////  targetPower = Math.max(0, Math.min(1, -controller.calculate(testRPM - curVelocity)));
                targetPower = Math.max(0, Math.min(1, (feedback+feedforward)*batteryCompensation));
                LeftShooter.setPower(targetPower);
                RightShooter.setPower(targetPower);

            } else {
//                stopShooter();
                stopShooter();
            }

//


            Angler.setPosition(hoodAngle);




            //cooked - near
//            if (graduatedDistance < 110) {
//                if (((testRPM - RcurrentRPM) > 150) && ((testRPM - RcurrentRPM) < 250)) {
//                    Kf2 = 12.05;
//                    Kp2 = 2.28;
//                    if (Kd2 >= 0.03) {
//                        Kd2 = Kd2 - 0.0001;
//                    }
//                } else if ((testRPM - RcurrentRPM) < -75) {
//                    Kf2 = 11.65;
//                    Kp2 = 1.85;
//                    Kd2 = Kd2 + 0.0004;
//                } else {
//                    Kf2 = 11.9;
//                    Kp2 = 2.12;
//                }
//            }
//            else if (graduatedDistance >= 110) {
//                if (((testRPM - RcurrentRPM) > 150) && ((testRPM - RcurrentRPM) < 250)) {
//                    Kf3 = 11.22;
//                    Kp3 = 2.28;
//                    Kd3 = Kd3 - 0.0001;
//                } else if ((testRPM - RcurrentRPM) < -75) {
//                    Kf3 = 10.95;
//                    Kp3 = 2.08;
//                    Kd3 = Kd3 + 0.0002;
//                } else {
//                    Kf3 = 11.15;
//                    Kp3 = 2.32;
//                }
//            }

//                telemetry.addData("Shooter", "ON → %.1f RPM", FAR_TARGET_RPM);
//                telemetry.addData("Error", "%.1f ticks/sec", FAR_TARGET_TICKS_PER_SEC - currentTPS);
//
            telemetry.addData("Distance", graduatedDistance);
            telemetry.addData("Shooter TestRPM", "ON → %.1f RPM", testRPM);

            telemetry.addData("Hood", Angler.getPosition());


//                telemetry.addData("Error R", "%.1f ticks/sec", testRPM * TICKS_PER_REV / 60.0 - RcurrentTPS);
            telemetry.addData("RPM-Error", testRPM - RcurrentRPM);
//                telemetry.addData("Actual Velocity", "%.1f ticks/sec", RcurrentTPS);
            telemetry.addData("Actual RPM Right", "%.1f", RcurrentRPM);
//                telemetry.addData("Actual TPS, ", "%.1f ticks/sec", RcurrentTPS);
//
//
//                telemetry.addData("Error L", "%.1f ticks/sec", testRPM * TICKS_PER_REV / 60.0 - LcurrentTPS);
            telemetry.addData("RPM-Error", testRPM - LcurrentRPM);
//                telemetry.addData("Actual Velocity", "%.1f ticks/sec", LcurrentTPS);
//            telemetry.addData("Actual RPM Left", "%.1f", LcurrentRPM);

            telemetry.addData("launcherOn",launcherOn);

            telemetry.addLine();


            telemetry.addData("Coefficients", controller.getCoefficients());

            telemetry.addData("P", P);
            telemetry.addData("I", I);
            telemetry.addData("kS", kS);
            telemetry.addData("kV", kV);
            telemetry.addLine();
            telemetry.addData("targetPower", targetPower);
            telemetry.addData("left power", LeftShooter.getPower());
            telemetry.addData("right power", RightShooter.getPower());


//            PanelsTelemetry.telemetry.addData("L rpm", LcurrentRPM);
//            panelsTelemetry.update(telemetry);


//            telemetryM.addData("LcurrentRPM", Double.valueOf(LcurrentRPM));
//            telemetryM.addData("RcurrentRPM", RcurrentRPM);
//            telemetryM.addData("testRPM", testRPM);
//            telemetryM.update();


//                telemetry.addData("Actual TPS, ", "%.1f ticks/sec", LcurrentTPS);
//                telemetry.addData("Color", RGBlight.getPosition());
//



            telemetry.update();
            }
        }
    }
}

