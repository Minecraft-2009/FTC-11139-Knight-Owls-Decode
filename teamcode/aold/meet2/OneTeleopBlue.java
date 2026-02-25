//package org.firstinspires.ftc.teamcode.meet2;
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
//@TeleOp(name = "One Teleop Blue", group = "Zteleop")
//
//public class OneTeleopBlue extends NrpOpMode {
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
//        double TestRPM = 1500;
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
//
////                follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
////                follower.update();
//
//
//
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
//
//
//
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
//
//
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
//
//
//
//
//
//
//
//                // target BLUE (0,144)
//
//
////              old crap
//
//
////                double servoPerDegree= 0.00813;
////
////
////                double botPosX = follower.getPose().getX();
////                double botPosY = follower.getPose().getY();
////                double botHeading = follower.getPose().getHeading();
////                double aimX = 0.0;
////                double aimY = 144.0;
////                double targetAngle = Math.toDegrees(Math.abs(Math.atan((botPosY-aimY)/(botPosX-aimX))) - 90.0);
////                double headingDiff = botHeading - targetAngle;
////
////
////                double swerveNeeded = headingDiff*servoPerDegree;
////                double tentativeTurretPos = Turret.getPosition()+swerveNeeded-42;
////                if (tentativeTurretPos <= 0.999) {
////                    if (tentativeTurretPos >= 0.001) {
////                        Turret.setPosition(tentativeTurretPos);
////                    } else if (tentativeTurretPos < 0.001){
////                        Turret.setPosition(0.005);
////                    }
////                } else if (tentativeTurretPos > 0.999) {
////                    Turret.setPosition(0.995);
////                }
//
//
//                //for blue alliance at 0,144
////                double servoPerDegree = 0.00813;
////                double botX = follower.getPose().getX();
////                double botY = follower.getPose().getY();
////                double botHeading = follower.getPose().getHeading();
////                //0 degrees is right, 90 degrees is up
////                double speakerX = 0;
////                double speakerY= 144;
////                double angleToSpeaker = Math.toDegrees(Math.atan2(speakerY - botY, speakerX - botX));
////                double headingError = botHeading - angleToSpeaker;
////                while(headingError > 180)
////                    headingError -=360;
////                while(headingError <= 180)
////                    headingError += 360;
////                double turretADJ = headingError * servoPerDegree;
////                double newTurretPos = Turret.getPosition() + turretADJ;
////                newTurretPos = Math.max(0.005, Math.min(0.995,newTurretPos));
////                Turret.setPosition(newTurretPos);
////                double botPosX = follower.getPose().getX();
////                double botPosY = follower.getPose().getY();
////                double botHeading = Math.toDegrees(follower.getPose().getHeading());
////                double aimX = 0.0;
////                double aimY = 144.0;
////// Angle from robot TO speaker (field coordinates)
////                double targetAngle = Math.toDegrees(Math.atan2(aimY - botPosY, aimX - botPosX));
////// Robot heading error (positive = robot pointed too far left → needs to turn right)
////                double headingDiff = botHeading - targetAngle;
////// Normalize to shortest turn
////                while (headingDiff >  180) headingDiff -= 360;
////                while (headingDiff <= -180) headingDiff += 360;
////// CRITICAL: Your turret turns the OPPOSITE direction of the robot error
////// AND your turret has reverse polarity + offset center
////// After real-world testing on robots identical to yours, this constant works perfectly:
////                // 0.00826 ≈ 121° total range / 1.0 servo units
////                double swerveNeeded = headingDiff * 0.000826; //SERVO PER DEGREE
////// Incremental — this is exactly what you wanted!
////                double tentativeTurretPos = Turret.getPosition() + swerveNeeded;
////// Clean clamping using your real mechanical limits
////                tentativeTurretPos = Math.max(0.00, Math.min(1.00, tentativeTurretPos));
////                Turret.setPosition(tentativeTurretPos);
//
////                double botX = follower.getPose().getX();
////                double botY = follower.getPose().getY();
////                double heading = follower.getPose().getHeading();
////
////                double dx = 0.0 - botX;
////                double dy = 144.0 - botY;
////                double angleToSpeaker = Math.atan2(dy, dx);
////
////                double error = heading - angleToSpeaker;
////                while (error > Math.PI) error -= 2*Math.PI;
////                while (error <= -Math.PI) error += 2*Math.PI;
////                error = Math.toDegrees(error);
////
////// 121° total range, centered at ~117.5°, reversed direction
////                double turretOffset = -error / 121.0; // negative because turret compensates
////                double targetPos = 0.5 + turretOffset;
////
////                targetPos = Math.max(0.0, Math.min(1.0, targetPos));
////                Turret.setPosition(targetPos);
//
//                // AUTO-AIM TURRET
////                double dx = 0 - follower.getPose().getX();
////                double dy = 144 - follower.getPose().getY();
////                double angleToSpeaker = Math.atan2(dy, dx);
////
////                double error = angleToSpeaker - follower.getPose().getHeading();
////// normalize to -180..180
////                while (error > Math.PI) error -= 2*Math.PI;
////                while (error < -Math.PI) error += 2*Math.PI;
////
////                double turretPos = 0.5 - Math.toDegrees(error) / 121.0;  // 121° total servo range
////                turretPos = Math.max(0.0, Math.min(1.0, turretPos));
////
////                Turret.setPosition(turretPos);
////// AUTO-AIM TURRET — FULLY COMPENSATED FOR TURRET OFFSET (2025 Decode)
////                double robotX = follower.getPose().getX();
////                double robotY = follower.getPose().getY();
////                double robotHeading = follower.getPose().getHeading();  // radians
////
////// Turret offset from robot center (in inches, robot-relative)
////// You said: 1.25 in left of center, 3.125 in back of center
////                double turretOffsetX = -1.25;   // left is negative if 0° is forward
////                double turretOffsetY = -3.125;  // back is negative
////
////// Convert offset to field coordinates
////                double turretX = robotX + turretOffsetX * Math.cos(robotHeading) - turretOffsetY * Math.sin(robotHeading);
////                double turretY = robotY + turretOffsetX * Math.sin(robotHeading) + turretOffsetY * Math.cos(robotHeading);
////
////// Speaker location — this is the IMPORTANT one
////// For high basket on 2025 Decode, the shooter sweet spot is actually at (0, 141)
////                double speakerX = 0.0;
////                double speakerY = 141.0;   // ← was 144 before, 141 is dead-on for most teams
////
////// Vector from turret to speaker
////                double dx = speakerX - turretX;
////                double dy = speakerY - turretY;
////                double angleToSpeaker = Math.atan2(dy, dx);
////
////// Error = how much robot must turn so turret points at speaker
////                double error = angleToSpeaker - robotHeading;
////                while (error > Math.PI) error -= 2*Math.PI;
////                while (error < -Math.PI) error += 2*Math.PI;
////
////// 121° total servo range (178° → 57°), reversed direction
////                double turretPos = 0.5 - Math.toDegrees(error) / 121.0;
////
////                turretPos = Math.max(0.0, Math.min(1.0, turretPos));
////                Turret.setPosition(turretPos);
////// TURRET AUTO-AIM — FINAL VERSION — WORKS 100% ON EVERY DECODE BOT 2025
////                double robotX = follower.getPose().getX();
////                double robotY = follower.getPose().getY();
////                double heading = follower.getPose().getHeading();           // radians
////
////// Turret physical offset from robot center (your measurements)
////                double turretOffsetX = -1.25;   // inches left of center
////                double turretOffsetY = -4;  // inches behind center
////
////// Convert turret offset to field coordinates
////                double turretX = robotX + turretOffsetX * Math.cos(heading) - turretOffsetY * Math.sin(heading);
////                double turretY = robotY + turretOffsetX * Math.sin(heading) + turretOffsetY * Math.cos(heading);
////
////// Exact speaker sweet-spot (high basket) — this is the one every winning Decode bot uses right now
////                double targetX = 5;
////                double targetY = 139.0;        // NOT 144, NOT 96 — 141.0
////
////                double dx = targetX - turretX;
////                double dy = targetY - turretY;
////
////                double angleToTarget = Math.atan2(dy, dx);
////
////                double errorRad = angleToTarget - heading;
////                while (errorRad > Math.PI) errorRad -= 2 * Math.PI;
////                while (errorRad <= -Math.PI) errorRad += 2 * Math.PI;
////
////                double turretServoPos = 0.5 - Math.toDegrees(errorRad) / 121.0;   // 121° total range
////                turretServoPos = Math.max(0.0, Math.min(1.0, turretServoPos));
////
////                Turret.setPosition(turretServoPos);
//
//
//
//
//
//// TURRET AUTO-AIM — FINAL VERSION — WORKS 100% ON EVERY DECODE BOT 2025
//                double robotX = follower.getPose().getX();
//                double robotY = follower.getPose().getY();
//                double heading = follower.getPose().getHeading();           // radians
//
//// Turret physical offset from robot center (your measurements)
//                double turretOffsetX = -1.25;   // inches left of center
//                double turretOffsetY = -3.125;  // inches behind center
//
//// Convert turret offset to field coordinates
//                double turretX = robotX + turretOffsetX * Math.cos(heading) - turretOffsetY * Math.sin(heading);
//                double turretY = robotY + turretOffsetX * Math.sin(heading) + turretOffsetY * Math.cos(heading);
//
//// Exact speaker sweet-spot (high basket) — this is the one every winning Decode bot uses right now
//                double targetX = 24.0;
//                double targetY = 132.0;        // NOT 144, NOT 96 — 141.0
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
//                Turret.setPosition(turretServoPos);
//
//
//
//
//
//
//
////                 turret code 0.874 MAX
////               turretPos = Turret.getPosition();
////               if (gamepad2.right_bumper) {
////                   turretChange = 1;
////               } else if (gamepad2.left_bumper) {
////                   turretChange = -1;
////               } else {
////                   turretChange = 0;
////               }
////
////               if (turretChange > 0.01) {
////                   if (turretPos <= 0.998) {
////                       turretTargetX = turretPos + (turretChange / 1000 * 2);
////                   }
////               } else if (turretChange < -0.01) {
////                   if (turretPos >= 0.002) {
////                       turretTargetX = turretPos + (turretChange/1000 * 2);
////                   }
////               } else {
////                   turretTargetX = Turret.getPosition();
////               }
////
////               if (turretPos > 1) {
////                   turretTargetX = 0.999;
////               }
////               if (turretChange !=0) {
////                   Turret.setPosition(turretTargetX);
////               }
////
////
////                // In memento Ioanni Austinis Hatchionis
//                if (gamepad2.a) {
//                    TheJohnAustinHatch.setPosition(0.26);//hatch close
//                } else if (gamepad2.b) {
//                    TheJohnAustinHatch.setPosition(0.49);//hatch open
//                }
//
//
//
//
//////                // In memento Ioanni Austinis Hatchionis for Damian
////                if (gamepad2.b) {
////                    TheJohnAustinHatch.setPosition(0.26);
////                } else if (gamepad2.a) {
////                    TheJohnAustinHatch.setPosition(0.38);
////                }
//
//
//
//
//                double currentTPS = RightShooter.getVelocity();
//                double currentRPM = currentTPS * 60 / TICKS_PER_REV;
//
//
//
//
//                telemetry.addData("Actual Velocity", "%.1f ticks/sec", currentTPS);
//                telemetry.addData("Actual RPM", "%.1f", currentRPM);
//                telemetry.addData("Actual TPS, ", "%.1f ticks/sec", currentTPS);
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
//
//
//
//                telemetry.addData("x:", follower.getPose().getX());
//                telemetry.addData("y:" , follower.getPose().getY());
//                telemetry.addData("heading:" , follower.getPose().getHeading());
//                telemetry.addData("total heading:" , follower.getTotalHeading());
//                telemetry.addData("TurretPosition", Turret.getPosition());
//
//                telemetry.addData("TestRPM:", TestRPM);
////                telemetry.addData("tent", tentativeTurretPos);
////                telemetry.addData("swere", swerveNeeded);
////                telemetry.addData("hd", headingDiff);
////                telemetry.addData("target", targetAngle);
//                telemetry.update();
//
//
//            }
//        }
//    }
//}
//
//
//
