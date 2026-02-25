//package org.firstinspires.ftc.teamcode.meet2;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.Pose;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.NrpOpMode;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//
//@TeleOp(name = "One Teleop Red", group = "Zteleop")
//
//public class OneTeleopRed extends NrpOpMode {
//
//    public static Follower follower;
//
//    double regularDivBy = 1;
//    boolean AutoTracking = true;
//
////   public void setTurretAngle(double target) {
////   double posOneAngle = 55.0;
////   double posZeroAngle = 178.0;
////   Turret.setPosition()
////
////   }
//
//    public void runOpMode() {
//        //init stuff
//        initControls();
//
//        //pedro turret stuff
//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(new Pose(72,72, Math.toRadians(90))); // not center`
//        follower.update();
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
////                follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
////                follower.update();
//
//                // Shoot (L&R opposite directions) (tune values)
//                if (gamepad2.dpad_up) {
//                    runShooterAtRPM(FAR_TARGET_RPM);
//                    farShot = true;
//                    nearShot = false;
//                }
//                if (gamepad2.dpad_left) {
//                    runShooterAtRPM(NEAR_TARGET_RPM);
//                    nearShot = true;
//                    farShot = false;
//                }
////                if (gamepad2.dpad_right) {
////                    LeftShooter.setPower(0.5);
////                    RightShooter.setPower(-0.5);
////                }
//                if (gamepad2.dpad_down) {
//                    stopShooter();
//                    nearShot = false;
//                    farShot = false;
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
////                // Intake for Damian
////                if (gamepad2.right_trigger != 0) {
////                    Intake.setPower(-gamepad2.right_trigger);
////                }else if (gamepad2.left_trigger != 0) {
////                    Intake.setPower(gamepad2.left_trigger*0.75);
////                }else {
////                    Intake.setPower(0);
////                }
//
//                double robotX = follower.getPose().getX();
//                double robotY = follower.getPose().getY();
//                double heading = follower.getPose().getHeading();           // radians
//
//                // Turret physical offset from robot center
//                double turretOffsetX = -1.25;   // inches left of center
//                double turretOffsetY = -3.125;  // inches behind center
//
//                // Convert turret offset to field coordinates
//                double turretX = robotX + turretOffsetX * Math.cos(heading) - turretOffsetY * Math.sin(heading);
//                double turretY = robotY + turretOffsetX * Math.sin(heading) + turretOffsetY * Math.cos(heading);
//
//                // Targets
//                double targetX = 141.0;
//                double targetY = 130.0;
//
//                double dx = targetX - turretX;
//                double dy = targetY - turretY;
//
//                double angleToTarget = Math.atan2(dy, dx);
//
//                double errorRad = angleToTarget - heading;
//                while (errorRad > Math.PI) errorRad -= 2 * Math.PI;
//                while (errorRad <= -Math.PI) errorRad += 2 * Math.PI;
//
//                double turretServoPos = 0.733 - Math.toDegrees(errorRad) / 120.0;   // 120° total range
//
//                turretServoPos = Math.max(0.0, Math.min(1.0, turretServoPos));
//
//                if (AutoTracking) {
//                    Turret.setPosition(turretServoPos);
//                } else if (!AutoTracking) {
////                 turret code 0.874 MAX
//                    turretPos = Turret.getPosition();
//                    if (gamepad2.right_bumper) {
//                        turretChange = 1;
//                    } else if (gamepad2.left_bumper) {
//                        turretChange = -1;
//                    } else {
//                        turretChange = 0;
//                    }
//
//                    if (turretChange > 0.01) {
//                        if (turretPos <= 0.998) {
//                            turretTargetX = turretPos + (turretChange / 1000 * 2);
//                        }
//                    } else if (turretChange < -0.01) {
//                        if (turretPos >= 0.002) {
//                            turretTargetX = turretPos + (turretChange / 1000 * 2);
//                        }
//                    } else {
//                        turretTargetX = Turret.getPosition();
//                    }
//
//                    if (turretPos > 1) {
//                        turretTargetX = 0.999;
//                    }
//                    if (turretChange != 0) {
//                        Turret.setPosition(turretTargetX);
//                    }
//                }
//
//                if (gamepad2.y) {
//                    AutoTracking = true;
//                }
//                if (gamepad2.x) {
//                    AutoTracking = false;
//                }
//
////                // In memento Ioanni Austinis Hatchionis
//                if (gamepad2.a) {
//                    TheJohnAustinHatch.setPosition(0.26);//hatch close
//                } else if (gamepad2.b) {
//                    TheJohnAustinHatch.setPosition(0.49);//hatch open
//                }
//
//////                // In memento Ioanni Austinis Hatchionis for Damian
////                if (gamepad2.b) {
////                    TheJohnAustinHatch.setPosition(0.26);
////                } else if (gamepad2.a) {
////                    TheJohnAustinHatch.setPosition(0.38);
////                }
//
//
//                double currentTPS = RightShooter.getVelocity();
//                double currentRPM = currentTPS * 60 / TICKS_PER_REV;
//
//
//
//
////                telemetry.addData("Actual Velocity", "%.1f ticks/sec", currentTPS);
////                telemetry.addData("Actual RPM", "%.1f", currentRPM);
////                telemetry.addData("Actual TPS, ", "%.1f ticks/sec", currentTPS);
////                if (farShot) {
////                    telemetry.addData("Shooter", "ON → %.1f RPM", FAR_TARGET_RPM);
////                    telemetry.addData("Error", "%.1f ticks/sec", FAR_TARGET_TICKS_PER_SEC - currentTPS);
////                }
////                if (nearShot) {
////                    telemetry.addData("Shooter", "ON → %.1f RPM", NEAR_TARGET_RPM);
////                    telemetry.addData("Error", "%.1f ticks/sec", NEAR_TARGET_TICKS_PER_SEC - currentTPS);
////                }
//                follower.update();
//
//                telemetry.addData("x:", follower.getPose().getX());
//                telemetry.addData("y:" , follower.getPose().getY());
//                telemetry.addData("heading:" , follower.getPose().getHeading());
//                telemetry.addData("total heading:" , follower.getTotalHeading());
//                telemetry.addData("TurretPosition", Turret.getPosition());
////                telemetry.addData("tent", tentativeTurretPos);
////                telemetry.addData("swere", swerveNeeded);
////                telemetry.addData("hd", headingDiff);
////                telemetry.addData("target", targetAngle);
//                telemetry.update();
//
//            }
//        }
//    }
//}
//
//
//
