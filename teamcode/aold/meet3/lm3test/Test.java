//package org.firstinspires.ftc.teamcode.aold.meet3.lm3test;
//
//
//
//import com.pedropathing.follower.Follower;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//@TeleOp(name = "TEST MEET THREE")
//
//public class Test extends NrpTestMode {
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
//        if (opModeIsActive()) {
//            // Wheel code
//            while (opModeIsActive()) {
////                if (gamepad2.left_stick_y != 0 || gamepad2.left_stick_x != 0 || gamepad2.right_stick_x != 0) {
////                    double r = Math.hypot(gamepad2.left_stick_x, gamepad2.left_stick_y);
////                    double robotAngle = Math.atan2(gamepad2.left_stick_y, gamepad2.left_stick_x) - Math.PI / 4;
////                    double turn = gamepad2.right_stick_x;
////                    double rightX = turn * turn * turn;
////                    final double v1 = (-r * Math.cos(robotAngle) + rightX);
////                    final double v2 = (-r * Math.sin(robotAngle) - rightX);
////                    final double v3 = (-r * Math.sin(robotAngle) + rightX);
////                    final double v4 = (-r * Math.cos(robotAngle) - rightX);
////                    LeftBack.setPower(v1 * regularDivBy);
////                    RightBack.setPower(v2 * regularDivBy);
////                    LeftFront.setPower(v3 * regularDivBy);
////                    RightFront.setPower(v4 * regularDivBy);
////
////
////
////
//////                    telemetry.addData("LB", v1 * regularDivBy);
//////                    telemetry.addData("RB", v2 * regularDivBy);
//////                    telemetry.addData("LF", v3 * regularDivBy);
//////                    telemetry.addData("RF", v4 * regularDivBy);
//////                    telemetry.update();
////                } else {
////                    LeftFront.setPower(0);
////                    LeftBack.setPower(0);
////                    RightFront.setPower(0);
////                    RightBack.setPower(0);
////                 }
//
//
////                follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
////                follower.update();
//
//
////
////                 In memento Ioanni Austinis Hatchionis
//                if (gamepad2.a) {
//                    TheJohnAustinHatch.setPosition(0);//hatch close
//                } else if (gamepad2.b) {
//                    TheJohnAustinHatch.setPosition(0.25);//hatch open
//                }
//
////
////                    // Angler Fish
////                    if (gamepad2.dpad_up) {
////                        Angler.setPosition(0.45);
////                    } else if (gamepad2.dpad_down) {
////                        Angler.setPosition(1);
////                    } else if (gamepad2.dpad_left) {
////                        Angler.setPosition(0.75);
////                    }
//
//                    // turret zeroing
////                if (gamepad2.dpad_left) {
////                    TurretL.setPosition(1);
////                }
////                if (gamepad2.dpad_right) {
////                    TurretR.setPosition(0);
////                }
//                if (gamepad2.dpad_down) {
//                    TurretL.setPosition(0.5);
//                    TurretR.setPosition(0.5);
//                }
////                if (gamepad2.dpad_up) {
////                    TurretL.setPosition(0.4);
////                    TurretR.setPosition(0.4);
////                }
//                if (gamepad2.dpad_left) {
//                    LeftShooter.setPower(-1);
//                    RightShooter.setPower(1);
//                }
//                if (gamepad2.dpad_right) {
//                    LeftShooter.setPower(0);
//                    RightShooter.setPower(0);
//                }
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
////                if (gamepad2.right_trigger >= 0.2) {
////                    if ((TurretL.getPosition() + 0.3) < 0.96) {
////                        TurretL.setPosition(TurretL.getPosition() + 0.03);
////                        TurretL.setPosition(TurretR.getPosition() + 0.3);
////                    }
////                } else if (gamepad2.left_trigger >= 0.2) {
////                    if ((TurretL.getPosition() - 0.3) > 0.04) {
////                        TurretL.setPosition(TurretL.getPosition() - 0.03);
////                        TurretL.setPosition(TurretR.getPosition() - 0.3);
////                    }
////                }
//
//
//
////
////                int targetX = 0;
////                int targetY = 144;
////
////                double robotX = follower.getPose().getX();
////                double robotY = follower.getPose().getY();
////                double robotHeadingDeg = Math.toDegrees(follower.getPose().getHeading());
////
////                double dx = targetX - robotX;
////                double dy = targetY - robotY;
////
////                double angleToTargetDeg = (Math.toDegrees(Math.atan2(dy, dx)) + 360) % 360;
////                double turretAngleDeg = ((angleToTargetDeg - robotHeadingDeg) + 540) % 360 - 180;
//
////                double pos = 0.5 + turretAngleDeg / 360.0;
//
////                turretLeft.setPosition(pos);
////                turretRight.setPosition(1.0 - pos);
//
//
//                    // PID Testing
//
////                double RcurrentTPS = RightShooter.getVelocity();
////                double RcurrentRPM = RcurrentTPS * 60 / TICKS_PER_REV;
////
////
////                double LcurrentTPS = LeftShooter.getVelocity();
////                double LcurrentRPM = LcurrentTPS * 60 / TICKS_PER_REV;
////
////
//////                telemetry.addData("Shooter", "ON → %.1f RPM", FAR_TARGET_RPM);
//////                telemetry.addData("Error", "%.1f ticks/sec", FAR_TARGET_TICKS_PER_SEC - currentTPS);
////
////                telemetry.addData("Shooter", "ON → %.1f RPM", TEST_RPM);
////
////                telemetry.addData("Error", "%.1f ticks/sec", TEST_TARGET_TICKS_PER_SEC - RcurrentTPS);
////                telemetry.addData("RPM-Error", TEST_RPM - RcurrentRPM);
////                telemetry.addData("Actual Velocity", "%.1f ticks/sec", RcurrentTPS);
////                telemetry.addData("Actual RPM", "%.1f", RcurrentRPM);
////                telemetry.addData("Actual TPS, ", "%.1f ticks/sec", RcurrentTPS);
////
////
////                telemetry.addData("Error", "%.1f ticks/sec", TEST_TARGET_TICKS_PER_SEC - LcurrentTPS);
////                telemetry.addData("RPM-Error", TEST_RPM - LcurrentRPM);
////                telemetry.addData("Actual Velocity", "%.1f ticks/sec", LcurrentTPS);
////                telemetry.addData("Actual RPM", "%.1f", LcurrentRPM);
////                telemetry.addData("Actual TPS, ", "%.1f ticks/sec", LcurrentTPS);
//
//                    telemetry.update();
//
//                }
//            }
//        }
//    }
//
