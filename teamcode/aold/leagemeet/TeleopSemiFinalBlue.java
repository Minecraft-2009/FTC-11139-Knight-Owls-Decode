package org.firstinspires.ftc.teamcode.aold.leagemeet;



import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.NrpOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Teleop Semi-Final Blue")

public class TeleopSemiFinalBlue extends NrpOpMode {

    public static Follower follower;

    double regularDivBy = 1;

    public void runOpMode() {
        //init stuff
        initControls();

        //pedro turret stuff
        follower = Constants.createFollower(hardwareMap);
        //start with end of auto
        follower.setStartingPose(new Pose(56.000, 109.000, Math.toRadians(150))); //TODO: Set for End of Auto
        follower.update();



        try {
            initCommands();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        waitForStart();

        boolean isGraduated = true;
        boolean isTracking = true;

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
                    follower.setPose(new Pose(135.3, 11.00, Math.toRadians(0)));
                    follower.update();
                }



                follower.update();
                Pose pose = follower.getPose();


                double targetX = 12;
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
                double pos = 0.25 + (1/(2*Math.PI))*Math.atan((-1*robotY+132)/(robotX-12)) + moddedRobotHeadingRad;
                pos = (pos-0.5)*360.0/355.0+0.5;
                if (pos > 1){
                    pos --;
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
                //TODO: Near graduation


//                //TODO: y= 0.000106346x^{4} - 0.0173809x^{3} + 0.666884x^{2} + 23.07246x + 1959.92984 = velocity // GET NEW EQUATION
                double graduatedNearRPM = 0.000106346*Math.pow(graduatedDistance, 4) - 0.0173809*Math.pow(graduatedDistance, 3) + 0.666884*Math.pow(graduatedDistance, 2) + 23.07246*graduatedDistance + 1959.92984;
//
//                //TODO: y= -0.000000214714x^{4} + 0.0000534017x^{3} - 0.00472863x^{2} + 0.172755x - 1.21858 = angle // GET NEW EQUATION
                double graduatedNearAngle = -0.000000214714*Math.pow(graduatedDistance, 4) + 0.0000534017*Math.pow(graduatedDistance, 3) - 0.00472863*Math.pow(graduatedDistance, 2) + 0.172755*graduatedDistance - 1.21858;

                if (graduatedNearAngle < 0.45) {
                    graduatedNearAngle = 0.45;
                }
                if (graduatedNearAngle > 1) {
                    graduatedNearAngle = 1;
                }

                if (graduatedDistance < 120.0) {


                    // Graduated stuff \\
                    if (isGraduated) {
                        Angler.setPosition(graduatedNearAngle);
                        telemetry.addData("Angling", "Yes");
                    }

                    if ((gamepad2.dpad_up) && (isGraduated)) {
                        runShooterAtRPM(graduatedNearRPM);
                    }
                    if ((gamepad2.dpad_down) && (isGraduated)) {
                        stopShooter();
                    }

                    //if dist < 40 {angle = 1}
                    if (graduatedDistance <= 40) {
                        graduatedNearAngle = 1.00;
                    }

                }


                double RcurrentTPS = RightShooter.getVelocity();
                double RcurrentRPM = RcurrentTPS * 60 / TICKS_PER_REV;
//

                double LcurrentTPS = LeftShooter.getVelocity();
                double LcurrentRPM = LcurrentTPS * 60 / TICKS_PER_REV;
//
                double currentRPM =  (RcurrentRPM + LcurrentRPM)/2;


                //TODO: Far graduation

                if (graduatedDistance >= 120) {

                    if (gamepad2.dpad_right) {
                        runShooterAtRPM(5000);
                        Angler.setPosition(0.55);
                    }
                    if (gamepad2.dpad_down) {
                        stopShooter();
                        Angler.setPosition(1);
                    }


////                // y= 0.000106346x^{4} - 0.0173809x^{3} + 0.666884x^{2} + 23.07246x + 1959.92984 = velocity
//                    double graduatedNearRPM = 0.000106346*Math.pow(graduatedDistance, 4)  - 0.0173809*Math.pow(graduatedDistance, 3) + 0.666884*Math.pow(graduatedDistance, 2) + 23.07246*graduatedDistance + 1959.92984;
////
////                // -0.000000214714x^{4} + 0.0000534017x^{3} - 0.00472863x^{2} + 0.172755x - 1.21858 = angle
//                    double graduatedNearAngle = -0.000000214714*Math.pow(graduatedDistance, 4) + 0.0000534017*Math.pow(graduatedDistance, 3) - 0.00472863*Math.pow(graduatedDistance, 2) + 0.172755*graduatedDistance + 1.21858;



//                    // Graduated stuff \\
//                    if (isGraduated) {
//                        Angler.setPosition(graduatedNearAngle);
//                    }
//
//                    if ((gamepad2.dpad_up) && (isGraduated)) {
//                        runShooterAtRPM(graduatedNearRPM);
//                    }
//                    if ((gamepad2.dpad_down) && (isGraduated)) {
//                        stopShooter();
//                    }
//
//

                }


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

                    }
                    if (gamepad2.dpad_up) { // mid
                        Angler.setPosition(0.875);
                        runShooterAtRPM(3400);
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
                telemetry.addData("grad near RPM", graduatedNearRPM);



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

