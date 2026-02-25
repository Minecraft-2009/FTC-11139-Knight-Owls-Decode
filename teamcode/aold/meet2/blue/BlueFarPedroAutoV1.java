//package org.firstinspires.ftc.teamcode.meet2.blue;
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.NrpOpMode;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//
//@Autonomous(name = "Blue Far Autonomous", group = "Blue")
//@Configurable
//public class BlueFarPedroAutoV1 extends NrpOpMode {  // ← Changed to LinearOpMode
//
//    private TelemetryManager panelsTelemetry;
//    private Follower follower;
//    private int pathState = 0;
//    private Paths paths;
//
//    // Optional: timers for delays
//    private ElapsedTime pathTimer = new ElapsedTime(); // Add this field at top!
//
//    @Override
//    public void runOpMode() {  // ← This replaces init() + loop()
//        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
//
//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(new Pose(56, 10, Math.toRadians(90))); // Blue close start
//
//        paths = new Paths(follower);
//
//
//        panelsTelemetry.debug("Status", "Initialized - Waiting for start");
//        panelsTelemetry.update(telemetry);
//
//        initControls();
//
//        waitForStart();                   // ← LinearOpMode requirement
//        pathTimer.reset();
//
//        panelsTelemetry.debug("Status", "Auto Started");
//        panelsTelemetry.update(telemetry);
//
//        // Main linear loop
//        while (opModeIsActive()) {
//            follower.update();            // ALWAYS call this every iteration
//            autonomousPathUpdate();
//
//            // Turret AutoLock (exactly the same as before)
//            double robotX = follower.getPose().getX();
//            double robotY = follower.getPose().getY();
//            double heading = follower.getPose().getHeading();
//            double turretOffsetX = -1.25;   // inches left of center
//            double turretOffsetY = -3.125;  // inches behind center
//            double turretX = robotX + turretOffsetX * Math.cos(heading) - turretOffsetY * Math.sin(heading);
//            double turretY = robotY + turretOffsetX * Math.sin(heading) + turretOffsetY * Math.cos(heading);
//            double targetX = 24.0;
//            double targetY = 132.0;
//            double dx = targetX - turretX;
//            double dy = targetY - turretY;
//            double angleToTarget = Math.atan2(dy, dx);
//            double errorRad = angleToTarget - heading;
//            while (errorRad > Math.PI) errorRad -= 2 * Math.PI;
//            while (errorRad <= -Math.PI) errorRad += 2 * Math.PI;
//            double turretServoPos = 0.733 - Math.toDegrees(errorRad) / 120.0;   // 120° total range
//            turretServoPos = Math.max(0.0, Math.min(1.0, turretServoPos));
//            Turret.setPosition(turretServoPos);
//
//            // Telemetry (unchanged)
//            panelsTelemetry.debug("Path State", pathState);
//            panelsTelemetry.debug("X", String.format("%.2f", follower.getPose().getX()));
//            panelsTelemetry.debug("Y", String.format("%.2f", follower.getPose().getY()));
//            panelsTelemetry.debug("Heading °", String.format("%.1f", Math.toDegrees(follower.getPose().getHeading())));
//            panelsTelemetry.update(telemetry);
//
//            telemetry.update();
//        }
//    }
//
//    public static class Paths {
//        // ← Everything inside Paths class is 100% identical
//        public PathChain line1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10, Path11;
//
//        public Paths(Follower follower) {
//            line1 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierLine(new Pose(56.000, 10.000), new Pose(56.000, 34.000))
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
//                    .build();
//        }
//    }
//
//
//    public void autonomousPathUpdate() {
//        switch (pathState) {
//            case 0:
//                runShooterAtRPM(FAR_TARGET_RPM);
//                pathState = 1;
//                break;
//
//            case 1: // Finished line1 → wait 0.5s → shoot → go to stack
//                if (!follower.isBusy()) {
//                    if (pathTimer.milliseconds() == 0) pathTimer.reset(); // first entry
//                    if (pathTimer.milliseconds() >= 3500) {
//                        shootThreeBalls();
//                        stopShooter();
//                        pathTimer.reset();
//                        pathState = 2;
//                    }
//                }
//                break;
//
//            case 2: // Finished line1 → wait 0.5s → shoot → go to stack
//                if (!follower.isBusy()) {
//                    if (pathTimer.milliseconds() >= 21000) {
//                        follower.followPath(paths.line1);
//                        pathState = 3;
//                    }
//                }
//                break;
//
//
//            case 3: // Park
//                if (!follower.isBusy()) {
//                    follower.breakFollowing();
//                    pathState = 99;
//                }
//                break;
//
//            case 99:
//                // Done
//                requestOpModeStop();
//                break;
//        }
//    }
//}