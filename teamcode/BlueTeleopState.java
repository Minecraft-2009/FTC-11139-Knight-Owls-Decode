package org.firstinspires.ftc.teamcode;



import static org.firstinspires.ftc.teamcode.Storage.getAutoEndPose;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.VoltageSensor;

//import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.NrpOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Blue Teleop State")

public class BlueTeleopState extends NrpOpMode {

    private VoltageSensor batteryVoltageSensor;


    public static Follower follower;

    double regularDivBy = 1;
    boolean shootingFar = false;
    boolean shootingNear = false;

    public void runOpMode() {
        //init stuff
        initControls();
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        double currentVoltage = 1;



        boolean TopSlot = false;
        boolean MiddleSlot = false;
        boolean FrontSlot = false;

        int totalSlots = 0;



        //pedro turret stuff
        follower = Constants.createFollower(hardwareMap);
        //start with end of auto
        follower.setStartingPose(getAutoEndPose()); //TODO: Set for End of Auto
        follower.update();




        try {
            initCommands();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }


        // Set Colour Variables
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

        // Top
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

        boolean isGraduated = true;
        boolean isTracking = true;
        boolean isLocked = false;

        double farRPM = 4500;
        double nearRPM = 3500;
        double lockedRPM = 3000;
        double targetRPM = 3000;

        if (opModeIsActive()) {
            // Wheel code
            while (opModeIsActive()) {



                currentVoltage = batteryVoltageSensor.getVoltage();



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
                if (gamepad1.x) {//corner [Square]
                    follower.setPose(new Pose(133.8477, 10.3381, Math.toRadians(0)));
                    follower.update();
                }
                if (gamepad1.y) { // auto init spot [Triangle]
//                    follower.setPose(new Pose(126.1, 120.80, Math.toRadians(40.7)));
                    follower.setPose(new Pose(18.5, 118.7, Math.toRadians(143)));

                    follower.update();
                }
                if (gamepad1.a) { // at gate [The X]
                    follower.setPose(new Pose(14.9, 70.5, Math.toRadians(180)));
                    follower.update();
                }
                if (gamepad1.b) { // exact auto start  [Circle]
                    follower.setPose(new Pose(18.5, 118.700, Math.toRadians(143)));

                    follower.update();
                }





                follower.update();
                Pose pose = follower.getPose();

                double targetX = 8;
                double targetY = 132;

                double robotX = follower.getPose().getX();
                double robotY = follower.getPose().getY();

                double robotHeadingRad = pose.getHeading();
                robotHeadingRad = (robotHeadingRad + 2*Math.PI) % (2*Math.PI);
                double robotHeadingDeg = robotHeadingRad * (180/Math.PI);

                double turretX = robotX - (2.5 * Math.cos(robotHeadingRad) );
                double turretY = robotY - (2.5 * Math.sin(robotHeadingRad) );

                double moddedRobotHeadingRad = robotHeadingRad/(2*Math.PI)-0.25;
                double pos = 0.25 + (1/(2*Math.PI))*Math.atan((-1*turretY+132)/(turretX-8)) + moddedRobotHeadingRad;
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






                double graduatedDistance = Math.sqrt((targetX-turretX)*(targetX-turretX) + (targetY-turretY)*(targetY-turretY)); // distance

                // y=0.000125806x^{4} - 0.0259667x^{3} + 1.70772x^{2} - 32.29572x + 3077.99517
                double graduatedNearRPM = 0.000125806*Math.pow(graduatedDistance, 4) - 0.0259667*Math.pow(graduatedDistance, 3) + 1.70772*Math.pow(graduatedDistance, 2) - 32.29572*graduatedDistance + 3077.99517;

//                //y= 0.0000000509332x^{4} - 0.0000184529x^{3} + 0.00241167x^{2} - 0.13161x + 3.29491
                double graduatedNearAngle = 0.0000000509332*Math.pow(graduatedDistance, 4) - 0.0000184529*Math.pow(graduatedDistance, 3) + 0.00241167*Math.pow(graduatedDistance, 2) - 0.13161*graduatedDistance + 3.29491;

                double graduatedFarAngle = -0.000000530303*Math.pow(graduatedDistance, 4) + 0.000294697*Math.pow(graduatedDistance, 3) - 0.06123674*Math.pow(graduatedDistance, 2) + 5.643525*graduatedDistance - 194.1875;

                graduatedFarAngle = Math.min(1.0, Math.max(0.45, graduatedFarAngle));
                graduatedNearAngle = Math.min(1.0, Math.max(0.45, graduatedNearAngle));

                graduatedNearRPM = Math.min(3550, Math.max(3000, graduatedNearRPM));

                if (graduatedDistance <= 37) {
                    graduatedNearAngle = 1;
                }


                if (gamepad2.dpad_up) {
                    shootingFar = false;
                    shootingNear = true;
                } else if (gamepad2.dpad_right) {
                    shootingFar = true;
                    shootingNear = false;
                } else if (gamepad2.dpad_down) {
                    shootingFar = false;
                    shootingNear = false;
                }


                if (isGraduated) { // GRADUATED SHOOTING
                    if ((shootingFar) && (!shootingNear)) {
                        runShooterAtRPM(farRPM, currentVoltage);
                        telemetry.addData("shooting far", "far");
                        Angler.setPosition(graduatedFarAngle);
                    } else if ((!shootingFar) && (shootingNear)) {
                        runShooterAtRPM(graduatedNearRPM, currentVoltage);
                        telemetry.addData("shooting near", "near");
                        Angler.setPosition(graduatedNearAngle);
                    }
                    if ((!shootingFar) && (!shootingNear)) {
                        stopShooter();
                        telemetry.addData("stopping shooter", "stop");
                    }
                }

                double RcurrentTPS = RightShooter.getVelocity();
                double RcurrentRPM = RcurrentTPS * 60 / TICKS_PER_REV;
//

                double LcurrentTPS = LeftShooter.getVelocity();
                double LcurrentRPM = LcurrentTPS * 60 / TICKS_PER_REV;
//
                double currentRPM =  (RcurrentRPM + LcurrentRPM)/2;


                // Manual Mode (isGraduated = false)

                if (!isGraduated) {
                    if (gamepad2.dpad_right) { //far
                        Angler.setPosition(0.525);
                        runShooterAtRPM(4750, currentVoltage);
                    }
                    if (gamepad2.dpad_up) { // mid
                        Angler.setPosition(0.825);
                        runShooterAtRPM(3300, currentVoltage);
                    }
                    if (gamepad2.dpad_left) { // near
                        Angler.setPosition(0.9);
                        runShooterAtRPM(3000, currentVoltage);
                    }
                    if (gamepad2.dpad_down) {
                        Angler.setPosition(1);
                        stopShooter();
                    }
                }






                //Color
                NormalizedRGBA LMcolors = RightMiddleCS.getNormalizedColors();
                NormalizedRGBA RMcolors = RightMiddleCS.getNormalizedColors();
                NormalizedRGBA LFcolors = RightFrontCS.getNormalizedColors();
                NormalizedRGBA RFcolors = RightFrontCS.getNormalizedColors();
                NormalizedRGBA Tcolors = TopCS.getNormalizedColors();



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



//
//                telemetry.addData("Top", TopSlot);
//                telemetry.addData("Middle", MiddleSlot);
//                telemetry.addData("Front", FrontSlot);
//                telemetry.addLine();
//
//




//                telemetry.addData("Distance", graduatedDistance);
//                telemetry.addData("grad near angle", graduatedNearAngle);
//
//                telemetry.addData("shootingFar", shootingFar);
//                telemetry.addData("shootingNear", shootingNear);
//
//                telemetry.addData("near rpm", graduatedNearRPM);
//                telemetry.addData("far angle", graduatedFarAngle);
//                telemetry.addLine();
//
//                telemetry.addData("isGraduated", isGraduated);
//                telemetry.addData("isTracking", isTracking);
//                telemetry.addData("Actual RPM Right", "%.1f", RcurrentRPM);
//                telemetry.addData("Actual RPM Left", "%.1f", LcurrentRPM);//
//
//                telemetry.addData("robot (x)", robotX);
//                telemetry.addData("robot (y)", robotY);
//                telemetry.addData("turret (x)", turretX);
//                telemetry.addData("turret (y)", turretY);

//                telemetry.addData("Heading Degrees", robotHeadingDeg);



                telemetry.update();

            }
        }
    }
}