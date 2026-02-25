package org.firstinspires.ftc.teamcode.testing;




import android.icu.text.UnicodeSetSpanner;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.TelemetryManager;


import org.firstinspires.ftc.teamcode.PIDFControllerNRP;

import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.sun.tools.javac.comp.Check;


import org.firstinspires.ftc.teamcode.NrpOpMode;
//import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@TeleOp(name = "Color Test", group = "ATest")

public class ColorTest extends NrpOpMode {


    public static Follower follower;



    double regularDivBy = 1;


    // slots
    boolean TopSlot = false;
    boolean MiddleSlot = false;
    boolean FrontSlot = false;

    int totalSlots = 0;

    public void runOpMode() {

        //init stuff
        initControls();

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


        //TODO: variables to know re balls


        //left middle
        double LMopenRed = 0.003;
        double LMopenGreen = 0.006;
        double LMopenBlue = 0.007;
        
        double LMGfilledRed = 0.001;
        double LMGfilledGreen = 0.002;
        double LMGfilledBlue = 0.003;

        double LMPfilledRed = 0.001;
        double LMPfilledGreen = 0.002;
        double LMPfilledBlue = 0.004;
        //Right middle
        double RMopenRed = 0.004;
        double RMopenGreen = 0.006;
        double RMopenBlue = 0.007;
        
        double RMGfilledRed = 0.001;
        double RMGfilledGreen = 0.002;
        double RMGfilledBlue = 0.003;

        double RMPfilledRed = 0.001;
        double RMPfilledGreen = 0.002;
        double RMPfilledBlue = 0.004;

        //left front
        double LFopenRed = 0.004;
        double LFopenGreen = 0.007;
        double LFopenBlue = 0.008;

        double LFGfilledRed = 0.002;
        double LFGfilledGreen = 0.003;
        double LFGfilledBlue = 0.006;

        double LFPfilledRed = 0.003;
        double LFPfilledGreen = 0.004;
        double LFPfilledBlue = 0.008;

        //RIGHT FRONT
        double RFopenRed = 0.004;
        double RFopenGreen = 0.007;
        double RFopenBlue = 0.008;

        double RFGfilledRed = 0.002;
        double RFGfilledGreen = 0.005;
        double RFGfilledBlue = 0.006;

        double RFPfilledRed = 0.003;
        double RFPfilledGreen = 0.004;
        double RFPfilledBlue = 0.008;

        //todo top
        double TopenRed = 0.003;
        double TopenGreen = 0.001;
        double TopenBlue = 0.001;
        
        double TGfilledRed = 0.004;
        double TGfilledGreen = 0.013;
        double TGfilledBlue = 0.010;

        double TPfilledRed = 0.006;
        double TPfilledGreen = 0.008;
        double TPfilledBlue = 0.013;




        waitForStart();

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


//

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

                NormalizedRGBA LMcolors = RightMiddleCS.getNormalizedColors();
                NormalizedRGBA RMcolors = RightMiddleCS.getNormalizedColors();
                NormalizedRGBA LFcolors = RightFrontCS.getNormalizedColors();
                NormalizedRGBA RFcolors = RightFrontCS.getNormalizedColors();
                NormalizedRGBA Tcolors = TopCS.getNormalizedColors();




                telemetry.addData("LMcolors", RMcolors);
                telemetry.addData("LM Red", "%.3f", LMcolors.red);
                telemetry.addData("LM Green", "%.3f", LMcolors.green);
                telemetry.addData("LM Blue", "%.3f", LMcolors.blue);
                telemetry.addLine();

                telemetry.addData("RMcolors", RMcolors);
                telemetry.addData("RM Red", "%.3f", RMcolors.red);
                telemetry.addData("RM Green", "%.3f", RMcolors.green);
                telemetry.addData("RM Blue", "%.3f", RMcolors.blue);
                telemetry.addLine();

                telemetry.addData("LFcolors", LFcolors);
                telemetry.addData("LF Red", "%.3f", LFcolors.red);
                telemetry.addData("LF Green", "%.3f", LFcolors.green);
                telemetry.addData("LF Blue", "%.3f", LFcolors.blue);
                telemetry.addLine();

                telemetry.addData("RFcolors", RFcolors);
                telemetry.addData("RF Red", "%.3f", RFcolors.red);
                telemetry.addData("RF Green", "%.3f", RFcolors.green);
                telemetry.addData("RF Blue", "%.3f", RFcolors.blue);
                telemetry.addLine();

                telemetry.addData("Tcolors", Tcolors);
                telemetry.addData("T Red", "%.3f", Tcolors.red);
                telemetry.addData("T Green", "%.3f", Tcolors.green);
                telemetry.addData("T Blue", "%.3f", Tcolors.blue);
                telemetry.addLine();

                telemetry.addData("Distance", graduatedDistance);


                FrontSlot = false;
                MiddleSlot = false;
                TopSlot = false;
                //Check Front//todo
                if ((LFcolors.red < LFopenRed) && (RFcolors.red < RFopenRed)) {
                    if ((LFcolors.green < LFopenGreen-0.001) && (RFcolors.green < RFopenGreen-0.001)) {
                        if ((LFcolors.blue <= LFopenBlue) && (RFcolors.blue <= RFopenBlue)) {//maybe delete
                            FrontSlot = true;
                        }
                    }
                }
                //Check Middle
                if ((LMcolors.red < LMopenRed) && (RMcolors.red < RMopenRed)) {
                    if ((LMcolors.green < LMopenGreen - 0.0015) && (RMcolors.green < RMopenGreen - 0.0015)) {
                        if ((LMcolors.blue < LMopenBlue - 0.001) && (RMcolors.blue < RMopenBlue - 0.001)) {
                            MiddleSlot = true;
                        }
                    }
                }

                //Check Top
                if (Tcolors.red > TopenRed) {
                    if (Tcolors.green > TopenGreen+0.001) {
                        if (Tcolors.blue > TopenBlue+0.001) {
                            TopSlot = true;
                        }
                    }
                }
                totalSlots = 0;
                if (TopSlot) {
                    totalSlots += 1;
                }
                if (MiddleSlot) {
                    totalSlots += 1;
                }
                if (FrontSlot) {
                    totalSlots += 1;
                }

                if (totalSlots == 0) {
                    RGBlight.setPosition(0.29);
                }
                if (totalSlots == 1) {
                    RGBlight.setPosition(0.388);
                }
                if (totalSlots == 2) {
                    RGBlight.setPosition(0.5);
                }
                if (totalSlots == 3) {
                    RGBlight.setPosition(0.611);
                }




                telemetry.addData("Top", TopSlot);
                telemetry.addData("Middle", MiddleSlot);
                telemetry.addData("Front", FrontSlot);


                telemetry.update();
            }
        }
    }
}

