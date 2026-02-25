//package org.firstinspires.ftc.teamcode.aold.meet3;
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
//@Autonomous(name = "Blue Close  LM3")
//@Configurable
//public class BlueClose3 extends NrpOpMode {
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
//        follower.setStartingPose(new Pose(22.400, 125.300, Math.toRadians(90))); // Blue close start
//
//        paths = new Paths(follower);
//
//
//        panelsTelemetry.debug("Status", "Initialized - Waiting for start");
//        panelsTelemetry.update(telemetry);
//
//        initControls();
//
//        waitForStart();
//        pathTimer.reset();
//
//        panelsTelemetry.debug("Status", "Auto Started");
//        panelsTelemetry.update(telemetry);
//
//        // Main linear loop
//        while (opModeIsActive()) {
//            follower.update();  // necessary
//            autonomousPathUpdate();
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
//        public PathChain Path1;
//        public PathChain Path2;
//        public PathChain Path3;
//        public PathChain Path4;
//        public PathChain Path5;
//        public PathChain Path6;
//        public PathChain Path7;
//        public PathChain Path8;
//        public PathChain Path9;
//        public PathChain Path10;
//        public PathChain Path11;
//
//        public Paths(Follower follower) {
//            Path1 = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    new Pose(22.400, 125.300),
//
//                                    new Pose(26.400, 140.000)
//                            )
//                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
//
//                    .build();
////
////            Path2 = follower.pathBuilder().addPath(
////                            new BezierLine(
////                                    new Pose(48.000, 104.900),
////
////                                    new Pose(42.500, 84.300)
////                            )
////                    ).setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180))
////
////                    .build();
////
////            Path3 = follower.pathBuilder().addPath(
////                            new BezierLine(
////                                    new Pose(42.500, 84.300),
////
////                                    new Pose(21.500, 84.300)
////                            )
////                    ).setConstantHeadingInterpolation(Math.toRadians(180))
////
////                    .build();
////
////            Path4 = follower.pathBuilder().addPath(
////                            new BezierLine(
////                                    new Pose(21.500, 84.300),
////
////                                    new Pose(49.000, 87.500)
////                            )
////                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(210))
////
////                    .build();
////
////            Path5 = follower.pathBuilder().addPath(
////                            new BezierLine(
////                                    new Pose(49.000, 87.500),
////
////                                    new Pose(42.500, 60.000)
////                            )
////                    ).setLinearHeadingInterpolation(Math.toRadians(210), Math.toRadians(180))
////
////                    .build();
////
////            Path6 = follower.pathBuilder().addPath(
////                            new BezierLine(
////                                    new Pose(42.500, 60.000),
////
////                                    new Pose(21.500, 60.000)
////                            )
////                    ).setConstantHeadingInterpolation(Math.toRadians(180))
////
////                    .build();
////
////            Path7 = follower.pathBuilder().addPath(
////                            new BezierLine(
////                                    new Pose(21.500, 60.000),
////
////                                    new Pose(59.000, 79.500)
////                            )
////                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(210))
////
////                    .build();
////
////            Path8 = follower.pathBuilder().addPath(
////                            new BezierLine(
////                                    new Pose(59.000, 79.500),
////
////                                    new Pose(42.500, 35.700)
////                            )
////                    ).setLinearHeadingInterpolation(Math.toRadians(210), Math.toRadians(180))
////
////                    .build();
////
////            Path9 = follower.pathBuilder().addPath(
////                            new BezierLine(
////                                    new Pose(42.500, 35.700),
////
////                                    new Pose(21.500, 35.700)
////                            )
////                    ).setConstantHeadingInterpolation(Math.toRadians(180))
////
////                    .build();
////
////            Path10 = follower.pathBuilder().addPath(
////                            new BezierLine(
////                                    new Pose(21.500, 35.700),
////
////                                    new Pose(59.000, 79.500)
////                            )
////                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(210))
////
////                    .build();
////
////            Path11 = follower.pathBuilder().addPath(
////                            new BezierLine(
////                                    new Pose(59.000, 79.500),
////
////                                    new Pose(21.000, 72.000)
////                            )
////                    ).setLinearHeadingInterpolation(Math.toRadians(210), Math.toRadians(180))
////
////                    .build();
//        }
//    }
//
//    public void autonomousPathUpdate() {
//        switch (pathState) {
//            case 0:
//                if (pathTimer.milliseconds() >= 25000) {
//                    follower.followPath(paths.Path1);
//                    pathState = 14;
//                }
//                break;
//
//
//            case 14: // Park
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