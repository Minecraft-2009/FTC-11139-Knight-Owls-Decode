package org.firstinspires.ftc.teamcode.aold.leagemeet;



import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.NrpOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Teleop Semi-Final Red")

public class TeleopSemiFinalRed extends NrpOpMode {

    public static Follower follower;

    double regularDivBy = 1;

    public void runOpMode() {
        //init stuff
        initControls();

        //pedro turret stuff
        follower = Constants.createFollower(hardwareMap);
        //start with end of auto
        follower.setStartingPose(new Pose(88.000, 109.000, Math.toRadians(30))); //TODO: Set for End of Auto
        follower.update();



        try {
            initCommands();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        waitForStart();

        boolean isGraduated = true;
        boolean isTracking = true;
        boolean isLocked = false;

        double farRPM = 4750;
        double nearRPM = 3300;
        double lockedRPM = 3000;
        double targetRPM = 3000;

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
                    follower.setPose(new Pose(8.700, 11.00, Math.toRadians(180)));
                    follower.update();
                }



                follower.update();
                Pose pose = follower.getPose();


                double targetX = 132;
                double targetY = 132;

                double robotX = follower.getPose().getX();
                double robotY = follower.getPose().getY();

                double robotHeadingRad = pose.getHeading();
                robotHeadingRad = (robotHeadingRad + 2*Math.PI) % (2*Math.PI);
                double robotHeadingDeg = robotHeadingRad * (180/Math.PI);

                double turretX = robotX - (2.5 * Math.cos(robotHeadingRad) );
                double turretY = robotY - (2.5 * Math.sin(robotHeadingRad) );

//
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


                if (isTracking) {
                    TurretL.setPosition(pos);
                    TurretR.setPosition(pos);
                } else if (!isTracking){
                    TurretL.setPosition(0.5);
                    TurretR.setPosition(0.5);
                }


                if ((gamepad2.y) && (!isGraduated)) {
                    if (isTracking) {
                        isTracking = false;
                    }
                    else if (!isTracking) {
                        isTracking = true;
                    }
                }


                if (gamepad2.x) {
                    if (isGraduated) {
                        isGraduated = false;
                    }
                    else if (!isGraduated) {
                        isGraduated = true;
                    }
                }



                double graduatedDistance = Math.sqrt((targetX-turretX)*(targetX-turretX) + (targetY-turretY)*(targetY-turretY)); // distance

//



//                // y=0.000125806x^{4} - 0.0259667x^{3} + 1.70772x^{2} - 32.29572x + 3077.99517
                double graduatedNearRPM = 0.000125806*Math.pow(graduatedDistance, 4) - 0.0259667*Math.pow(graduatedDistance, 3) + 1.70772*Math.pow(graduatedDistance, 2) - 32.29572*graduatedDistance + 3077.99517;
//
//                //y= 0.0000000509332x^{4} - 0.0000184529x^{3} + 0.00241167x^{2} - 0.13161x + 3.29491
                double graduatedNearAngle = 0.0000000509332*Math.pow(graduatedDistance, 4) - 0.0000184529*Math.pow(graduatedDistance, 3) + 0.00241167*Math.pow(graduatedDistance, 2) - 0.13161*graduatedDistance + 3.29491;

                if (graduatedNearAngle < 0.45) {
                    graduatedNearAngle = 0.45;
                }
                if (graduatedNearAngle > 1) {
                    graduatedNearAngle = 1;
                }


                //TODO: Far Graduation for ANGLE ONLY
                //TODO: y= - 0.000000530303x^{4} + 0.000294697x^{3} - 0.06123674x^{2} + 5.643525x - 194.1875
                //TODO: Anthony update this make sure its right

                double graduatedFarAngle = -0.000000530303*Math.pow(graduatedDistance, 4) + 0.000294697*Math.pow(graduatedDistance, 3) - 0.06123674*Math.pow(graduatedDistance, 2) + 5.643525*graduatedDistance - 194.1875;

                if (graduatedFarAngle < 0.45) {
                    graduatedFarAngle = 0.45;
                }
                if (graduatedFarAngle > 1) {
                    graduatedFarAngle = 1;
                }


                // Shooting


                // Graduated stuff \\

//                if (isGraduated) {
//                    Angler.setPosition(graduatedFarAngle);
//                    telemetry.addData("Angling", "Yes");
//                }

                if ((gamepad2.dpad_up) && (isGraduated)) {
                    isLocked = false;
                    // targetRPM = graduatedNearRPM;
                    runShooterAtRPM(3600);
                    Angler.setPosition(0.9);
                    // telemetry.addData("targetRPM", nearRPM);
                }
                if ((gamepad2.dpad_right) && (isGraduated)) {
                    isLocked = false;
                    runShooterAtRPM(4600);
                    Angler.setPosition(graduatedFarAngle);
                    // targetRPM = farRPM;
                    // telemetry.addData("targetRPM", farRPM);
                }
//                if ((gamepad2.dpad_left) && (isGraduated)) {
//                    isLocked = true;
//                    runShooterAtRPM(3000);
//                    Angler.setPosition(1);
//                    // lockedRPM = targetRPM;
//                    // telemetry.addData("targetRPM", lockedRPM);
//                }
                if ((gamepad2.dpad_down) && (isGraduated)) {
                    isLocked = false;
                    stopShooter();
                    telemetry.addData("targetRPM", 0);
                }

                // if (isLocked) {
                //     runShooterAtRPM(lockedRPM);
                // } else {
                //     runShooterAtRPM(targetRPM);
                // }


                double RcurrentTPS = RightShooter.getVelocity();
                double RcurrentRPM = RcurrentTPS * 60 / TICKS_PER_REV;
//

                double LcurrentTPS = LeftShooter.getVelocity();
                double LcurrentRPM = LcurrentTPS * 60 / TICKS_PER_REV;
//
                double currentRPM =  (RcurrentRPM + LcurrentRPM)/2;




                // Graduated Controls

//                if (gamepad2.x) {
//                    if (isGraduated) {
//                        isGraduated = false;
//                    }
//                    else if (!isGraduated) {
//                        isGraduated = true;
//                    }
//                }
//
//                // Graduated stuff \\
//                if (isGraduated) {
//                    Angler.setPosition(graduatedAngle);
//                }
//
//                if ((gamepad2.dpad_up) && (isGraduated)) {
//                    runShooterAtRPM(graduatedRPM);
//                }
//                if ((gamepad2.dpad_down) && (isGraduated)) {
//                    stopShooter();
//                }


                // Manual Mode (isGraduated = false)

                if (!isGraduated) {
//                    if (gamepad2.xWasPressed()) {
//                        testAngle = testAngle - 0.025;
//                        if (testAngle < 0.45) {
//                            testAngle = 0.45;
//                        }
//                        Angler.setPosition(testAngle);
//                    }
//                    if (gamepad2.yWasPressed()) {
//                        testAngle = testAngle + 0.025;
//                        if (testAngle > 1) {
//                            testAngle = 1;
//                        }
//                        Angler.setPosition(testAngle);
//                    }

                    if (gamepad2.dpad_right) { //far
                        Angler.setPosition(0.525);
                        runShooterAtRPM(4600);
                    }
                    if (gamepad2.dpad_up) { // mid
                        Angler.setPosition(0.875);
                        runShooterAtRPM(3600);
                    }
                    if (gamepad2.dpad_left) { // near
                        Angler.setPosition(1);
                        runShooterAtRPM(3000);
                    }

                    if (gamepad2.dpad_down) {
                        Angler.setPosition(1);
                        stopShooter();
                    }

                }

                telemetry.addData("grad near angle", graduatedNearAngle);


                telemetry.addData("isGraduated", isGraduated);
                telemetry.addData("isTracking", isTracking);
//                if (isGraduated) {
//                    telemetry.addData("graduatedRPM", graduatedNearRPM);
//                    telemetry.addData("graduatedAngle", graduatedNearAngle);
//                }
//                if (!isGraduated){
//                    telemetry.addData("Shooter TestRPM", "ON → %.1f RPM", testRPM);
//                    telemetry.addData("Test Angle", testAngle);
//                }
//
//                telemetry.addData("Error R", "%.1f ticks/sec", testRPM * TICKS_PER_REV / 60.0 - RcurrentTPS);
//                telemetry.addData("RPM-Error", testRPM - RcurrentRPM);
//                telemetry.addData("Actual Velocity", "%.1f ticks/sec", RcurrentTPS);
                telemetry.addData("Actual RPM Right", "%.1f", RcurrentRPM);
//                telemetry.addData("Actual TPS, ", "%.1f ticks/sec", RcurrentTPS);
//
//
//                telemetry.addData("Error L", "%.1f ticks/sec", testRPM * TICKS_PER_REV / 60.0 - LcurrentTPS);
//                telemetry.addData("RPM-Error", testRPM - LcurrentRPM);
//                telemetry.addData("Actual Velocity", "%.1f ticks/sec", LcurrentTPS);
                telemetry.addData("Actual RPM Left", "%.1f", LcurrentRPM);
//                telemetry.addData("Actual TPS, ", "%.1f ticks/sec", LcurrentTPS);
//

                telemetry.addData("robot (x)", robotX);
                telemetry.addData("robot (y)", robotY);
                telemetry.addData("turret (x)", turretX);
                telemetry.addData("turret (y)", turretY);

                telemetry.addData("","");
                telemetry.addData("Heading Degrees", robotHeadingDeg);

//                telemetry.addData("turX-botX", turretX-robotX);
//                telemetry.addData("turY-botY", turretY-robotY);
                telemetry.addData("Distance", graduatedDistance);


                telemetry.update();

            }
        }
    }
}