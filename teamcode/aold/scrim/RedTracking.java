//package org.firstinspires.ftc.teamcode.aold;
//
//
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.Pose;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.NrpOpMode;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//
//@TeleOp(name = "RedTracking")
//
//public class RedTracking extends NrpOpMode {
//
//    public static Follower follower;
//
//    double regularDivBy = 1;
//
////   public void setTurretAngle(double target) {
////   double posOneAngle = 55.0;
////   double posZeroAngle = 178.0;
////   Turret.setPosition()
////
////   }
//
//
//    public void runOpMode() {
//        //init stuff
//        initControls();
//
//
//        //pedro turret stuff
//        follower = Constants.createFollower(hardwareMap);
////        follower.setStartingPose(new Pose(72,72, Math.toRadians(90))); // not center`
//        //start with end of auto
//        follower.setStartingPose(new Pose(72.000, 72.000, Math.toRadians(90)));
//        follower.update();
//
//
//
//        try {
//            initCommands();
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }
//
//        waitForStart();
//        boolean farShot = false;
//        boolean nearShot = false;
//        double TestRPM = 1500;
//
//
//
//        if (opModeIsActive()) {
//            // Wheel code
//            while (opModeIsActive()) {
//                if (gamepad2.left_stick_y != 0 || gamepad2.left_stick_x != 0 || gamepad2.right_stick_x != 0) {
//                    double r = Math.hypot(gamepad2.left_stick_x, gamepad2.left_stick_y);
//                    double robotAngle = Math.atan2(gamepad2.left_stick_y, gamepad2.left_stick_x) - Math.PI / 4;
//                    double turn = gamepad2.right_stick_x;
//                    double rightX = turn * turn * turn;
//                    final double v1 = (-r * Math.cos(robotAngle) + rightX);
//                    final double v2 = (-r * Math.sin(robotAngle) - rightX);
//                    final double v3 = (-r * Math.sin(robotAngle) + rightX);
//                    final double v4 = (-r * Math.cos(robotAngle) - rightX);
//                    LeftBack.setPower(v1 * regularDivBy);
//                    RightBack.setPower(v2 * regularDivBy);
//                    LeftFront.setPower(v3 * regularDivBy);
//                    RightFront.setPower(v4 * regularDivBy);
//
//
//
////                    telemetry.addData("LB", v1 * regularDivBy);
////                    telemetry.addData("RB", v2 * regularDivBy);
////                    telemetry.addData("LF", v3 * regularDivBy);
////                    telemetry.addData("RF", v4 * regularDivBy);
////                    telemetry.update();
//                } else {
//                    LeftFront.setPower(0);
//                    LeftBack.setPower(0);
//                    RightFront.setPower(0);
//                    RightBack.setPower(0);
//                }
//
//
//
// /*
//
//                Gamepad 1:
//                dpad up/down/left: Angler
//
//                Gamepad 2:
//                dpad up/down: PID shooter on/off
//                a/b: John Auston Hatch
//                triggers: intake (in/out)
//                bumpers: transfer (in/out)
//                x: Turret @ 0.5
//
//                 */
//
//
//
//
////                 In memento Ioanni Austinis Hatchionis
//                if (gamepad2.a) {
//                    TheJohnAustinHatch.setPosition(0);//hatch close
//                } else if (gamepad2.b) {
//                    TheJohnAustinHatch.setPosition(0.25);//hatch open
//                }
//
//
//                // Angler Fish
//                if (gamepad1.dpad_right) {
//                    Angler.setPosition(0.45);
//                } else if (gamepad1.dpad_down) {
//                    Angler.setPosition(1);
//                } else if (gamepad1.dpad_left) {
//                    Angler.setPosition(0.875);
//
//                } else if (gamepad1.dpad_up) {
//                Angler.setPosition(0.75);
//                }
//                //
////
//                if (gamepad2.dpad_up) {
//                    Angler.setPosition(0.850);
//                    runShooterAtRPM(3500);
//                }
//                if (gamepad2.dpad_down) {
//                    Angler.setPosition(1);
//                    TheJohnAustinHatch.setPosition(0);
//                    stopShooter();
//                }
//                if (gamepad2.dpad_right) {
//                    Angler.setPosition(0.5040);
//                    runShooterAtRPM(5500);
//                }
//                if (gamepad2.dpad_left) {
//                    Angler.setPosition(1);
//                    runShooterAtRPM(3000);
//                }
//                if (gamepad2.y) {
//                    Angler.setPosition(0.775);
//                    runShooterAtRPM(4000);
//                }
//
//
////
////
////                double Rvelocity = RightShooter.getVelocity();
////                telemetry.addData("right", Rvelocity);
////                double Lvelocity = LeftShooter.getVelocity();
////                telemetry.addData("left", Lvelocity);
//
//                // Intake
//                if (gamepad2.right_trigger >= 0.2) {
//                    if (gamepad2.right_trigger >= 0.2) {
//                        Intake.setPower(-1);
//                    } else {
//                        Intake.setPower(0);
//                    }
//                } else {
//                    if (gamepad2.left_trigger >= 0.2) {
//                        Intake.setPower(0.5);
//                    } else {
//                        Intake.setPower(0);
//                    }
//                }
//
//                //Transfer
//                if (gamepad2.right_bumper) {
//                    Transfer.setPower(-1);
//                } else if (gamepad2.left_bumper) {
//                    Transfer.setPower(0.4);
//                } else {
//                    Transfer.setPower(0);
//                }
//
//
//                // location reset
//                if (gamepad1.x) {
//                    follower.setStartingPose(new Pose(8.500, 6.00, Math.toRadians(180)));
//                    follower.update();
//                }
//
//
//
//                follower.update();
//                Pose pose = follower.getPose();
//
//
//                double targetX = 140;
//                double targetY = 140;
//
//                double robotX = follower.getPose().getX();
//                double robotY = follower.getPose().getY();
//                double robotHeadingRad = pose.getHeading();
//                robotHeadingRad = (robotHeadingRad + 2*Math.PI) % (2*Math.PI);
//
//                double turretX = robotX - (2.5 * Math.cos(robotHeadingRad) );
//                double turretY = robotY - (2.5 * Math.sin(robotHeadingRad) );
//
//
//                double moddedRobotHeadingRad = robotHeadingRad/(2*Math.PI)-0.25;
//                double pos = 0.75 - (1/(2*Math.PI))*Math.atan((-1*turretY+132)/(-1*turretX+132)) + moddedRobotHeadingRad;
//                pos = (pos-0.5)*360.0/355.0+0.5;
//                if (pos > 1){
//                    pos --;
//                }
//
//                pos = Math.max(0.0, Math.min(1.0, pos));
//
//                TurretL.setPosition(pos);
//                TurretR.setPosition(pos);
//
//
//                double graduatedDistance = Math.sqrt((targetX-turretX)*(targetX-turretX) + (targetY-turretY)*(targetY-turretY)); // distance
//
//                /*
//                3500, (8.5) --> 74
//                 */
//
//
//
//
//
//
//                //TODO: NOTHING THIS IS JUST TO SAY THAT THIS IS THE END OF THE ACTUAL TELEOP
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
////
////                if (gamepad1.dpad_up) {
////                    follower = Constants.createFollower(hardwareMap);
////                    follower.setStartingPose(new Pose(8.75,6.375, Math.toRadians(180))); // not center`
////                    follower.update();
////                }
////
////
//////                follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
//////                follower.update();
////
////
////
//////                 In memento Ioanni Austinis Hatchionis
////                if (gamepad2.a) {
////                    TheJohnAustinHatch.setPosition(0);//hatch close
////                } else if (gamepad2.b) {
////                    TheJohnAustinHatch.setPosition(0.25);//hatch open
////                }
////
////
//////                    // Angler Fish
//////                    if (gamepad1.dpad_up) {
//////                        Angler.setPosition(0.45);
//////                    } else if (gamepad1.dpad_down) {
//////                        Angler.setPosition(1);
//////                    } else if (gamepad1.dpad_left) {
//////                        Angler.setPosition(0.75);
//////                    }
////
////                // turret zeroing
//////                if (gamepad2.dpad_left) {
//////                    TurretL.setPosition(1);
//////                }
//////                if (gamepad2.dpad_right) {
//////                    TurretR.setPosition(0);
//////                }
////                if (gamepad2.dpad_left) {
////                    LeftShooter.setVelocity(4000 * TICKS_PER_REV / 60.0);
////                    LeftShooter.setVelocityPIDFCoefficients(Kp, Ki, Kd, Kf); // P, I, D, F
////                }
////                if (gamepad2.dpad_up) {
////                    LeftShooter.setVelocity(3500 * TICKS_PER_REV / 60.0);
////                    LeftShooter.setVelocityPIDFCoefficients(Kp, Ki, Kd, Kf); // P, I, D, F
////                    RightShooter.setVelocity(3500 * TICKS_PER_REV / 60.0);
////                    RightShooter.setVelocityPIDFCoefficients(Kp, Ki, Kd, Kf); // P, I, D, F
////                }
////                if (gamepad2.dpad_right) {
////                    RightShooter.setVelocity(1000);
////                    RightShooter.setVelocityPIDFCoefficients(Kp, Ki, Kd, Kf); // P, I, D, F
////                }
////                if (gamepad2.dpad_down) {
////                    RightShooter.setVelocity(0);
////                    RightShooter.setVelocityPIDFCoefficients(Kp, Ki, Kd, Kf); // P, I, D, F
////                }
////
////                // Intake
////                if (gamepad2.right_trigger >= 0.2) {
////                    if (gamepad2.right_trigger >= 0.2) {
////                        Intake.setPower(-1);
////                    } else {
////                        Intake.setPower(0);
////                    }
////                } else {
////                    if (gamepad2.left_trigger >= 0.2) {
////                        Intake.setPower(0.5);
////                    } else {
////                        Intake.setPower(0);
////                    }
////                }
////
////                //Transfer
////                if (gamepad2.right_bumper) {
////                    Transfer.setPower(-1);
////                } else if (gamepad2.left_bumper) {
////                    Transfer.setPower(0.4);
////                } else {
////                    Transfer.setPower(0);
////                }
//
//
////                follower.update();
////
////                double targetX = 144;
////                double targetY = 144;
////
////                double robotX = follower.getPose().getX();
////                double robotY = follower.getPose().getY();
////                double robotHeadingDeg = Math.toDegrees(follower.getPose().getHeading());
////                robotHeadingDeg = (robotHeadingDeg + 360) % 360;
////                double dx = targetX - robotX;
////                double dy = targetY - robotY;
////                double angleToTargetDeg = Math.toDegrees(Math.atan2(dy, dx));
////                angleToTargetDeg = (angleToTargetDeg + 360) % 360;
////                double turretAngleDeg = angleToTargetDeg - robotHeadingDeg;
////                turretAngleDeg = ((turretAngleDeg + 540) % 360) - 180;
////                double TURRET_RANGE_DEG = 340.0;
////                double HALF_RANGE = TURRET_RANGE_DEG / 2.0;
////                turretAngleDeg = Math.max(-HALF_RANGE, Math.min(HALF_RANGE, turretAngleDeg));
////                double pos = 0.5 - (turretAngleDeg / TURRET_RANGE_DEG);
////                double servoCenter = 0.5;
////                double servoPerDegree = 0.00325;
//
////                double pos = (1/(2*Math.PI))*Math.atan((-1*robotY+144)/(-1*robotX+144)) + 0.5;
////
////                pos = Math.max(0.0, Math.min(1.0, pos));
////
////                TurretL.setPosition(pos);
////                TurretR.setPosition(pos);
//
////                Pose pose = follower.getPose();
////
////                double robotX = pose.getX();
////                double robotY = pose.getY();
////                double robotHeadingDeg = Math.toDegrees(pose.getHeading());
////                robotHeadingDeg = (robotHeadingDeg + 360) % 360;
////
////                double dx = targetX - robotX;
////                double dy = targetY - robotY;
////
////                double targetAngleDeg = Math.toDegrees(Math.atan2(dy, dx));
////                targetAngleDeg = (targetAngleDeg + 360) % 360;
////
////                double turretAngleDeg = targetAngleDeg - robotHeadingDeg;
////
////                turretAngleDeg = ((turretAngleDeg + 540) % 360) - 180;
////
////                final double TURRET_RANGE_DEG = 340.0;
////                final double HALF_RANGE = TURRET_RANGE_DEG / 2.0;
////
////                turretAngleDeg = Math.max(-HALF_RANGE, Math.min(HALF_RANGE, turretAngleDeg));
////
////                double servoPos = 0.5 - (turretAngleDeg / TURRET_RANGE_DEG);
////
////                servoPos = Math.max(0.0, Math.min(1.0, servoPos));
////
////                TurretL.setPosition(servoPos);
////                TurretR.setPosition(servoPos);
//
//
////                telemetry.addData("BotX", robotX);
////                telemetry.addData("BotY", robotY);
////                telemetry.addData("BotHead", robotHeadingDeg);
////                telemetry.addData("dx", dx);
////                telemetry.addData("dy", dy);
////
////                telemetry.addData("angleToTargetDeg", turretAngleDeg);
////                telemetry.addData("TurretAbsDeg", turretAngleDeg);
////                telemetry.addData("ServoPos", pos);
//
//
//
//
//
//
////
////
////
////
////                // PID Testing
////
//                double RcurrentTPS = RightShooter.getVelocity();
//                double RcurrentRPM = RcurrentTPS * 60 / TICKS_PER_REV;
////
//
//                double LcurrentTPS = LeftShooter.getVelocity();
//                double LcurrentRPM = LcurrentTPS * 60 / TICKS_PER_REV;
////
//
////                telemetry.addData("Shooter", "ON → %.1f RPM", FAR_TARGET_RPM);
////                telemetry.addData("Error", "%.1f ticks/sec", FAR_TARGET_TICKS_PER_SEC - currentTPS);
//
//                telemetry.addData("Shooter", "ON → %.1f RPM", TEST_RPM);
//
//                telemetry.addData("Error R", "%.1f ticks/sec", 3500 * TICKS_PER_REV / 60.0 - RcurrentTPS);
//                telemetry.addData("RPM-Error", TEST_RPM - RcurrentRPM);
//                telemetry.addData("Actual Velocity", "%.1f ticks/sec", RcurrentTPS);
//                telemetry.addData("Actual RPM", "%.1f", RcurrentRPM);
//                telemetry.addData("Actual TPS, ", "%.1f ticks/sec", RcurrentTPS);
//
//
//                telemetry.addData("Error L", "%.1f ticks/sec", 3500 * TICKS_PER_REV / 60.0 - LcurrentTPS);
//                telemetry.addData("RPM-Error", TEST_RPM - LcurrentRPM);
//                telemetry.addData("Actual Velocity", "%.1f ticks/sec", LcurrentTPS);
//                telemetry.addData("Actual RPM", "%.1f", LcurrentRPM);
////                telemetry.addData("Actual TPS, ", "%.1f ticks/sec", LcurrentTPS);
//
//
//                telemetry.addData("(x)", robotX);
//                telemetry.addData("(y)", robotY);
//                telemetry.addData("Distance", graduatedDistance);
//
//
//                telemetry.update();
//
//            }
//        }
//    }
//}
//
