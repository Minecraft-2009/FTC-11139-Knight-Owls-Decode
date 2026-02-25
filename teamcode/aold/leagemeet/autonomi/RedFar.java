package org.firstinspires.ftc.teamcode.aold.leagemeet.autonomi;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.NrpOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Red Far")
@Configurable
public class RedFar extends NrpOpMode {

    private TelemetryManager panelsTelemetry;
    private Follower follower;
    private int pathState = 0;
    private Paths paths;

    private VoltageSensor batteryVoltageSensor;

    double currentVoltage = 0;


    double targetX = 130;
    double targetY = 132;

    // Optional: timers for delays
    private ElapsedTime pathTimer = new ElapsedTime();

    boolean pathStarted = false;

    @Override
    public void runOpMode() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(88.0, 9, Math.toRadians(90)));

        paths = new Paths(follower);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();


        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0); // april tag detection

        /*
         * Starts polling for data.
         */
        limelight.start();



        panelsTelemetry.debug("Status", "Initialized - Waiting for start");
        panelsTelemetry.update(telemetry);

        initControls();

        waitForStart();
        pathTimer.reset();

        panelsTelemetry.debug("Status", "Auto Started");
        panelsTelemetry.update(telemetry);


        // Main linear loop
        while (opModeIsActive()) {
            follower.update();  // necessary


            // Turret AutoLock
            Pose currentPose = follower.getPose();


            double robotX = follower.getPose().getX();
            double robotY = follower.getPose().getY();
            double robotHeadingRad = currentPose.getHeading();
            robotHeadingRad = (robotHeadingRad + 2*Math.PI) % (2*Math.PI);
            double robotHeadingDeg = robotHeadingRad * (180/Math.PI);
            double turretX = robotX - (2.5 * Math.cos(robotHeadingRad) );
            double turretY = robotY - (2.5 * Math.sin(robotHeadingRad) );
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

            TurretL.setPosition(pos);
            TurretR.setPosition(pos);

            currentVoltage = batteryVoltageSensor.getVoltage();


            double graduatedDistance = Math.sqrt((targetX-turretX)*(targetX-turretX) + (targetY-turretY)*(targetY-turretY)); // distance

            double graduatedFarAngle = -0.000000530303*Math.pow(graduatedDistance, 4) + 0.000294697*Math.pow(graduatedDistance, 3) - 0.06123674*Math.pow(graduatedDistance, 2) + 5.643525*graduatedDistance - 194.1875;

            graduatedFarAngle = Math.min(1.0, Math.max(0.45, graduatedFarAngle));


            Angler.setPosition(graduatedFarAngle);


            //limelighy

            LLResult result;


            //happens after april tag motif detection
            limelight.pipelineSwitch(1);
            double PLtx, PLty, PLta, PCtx, PCty, PCta, PRtx, PRty, PRta, GLtx, GLty, GLta, GCtx, GCty, GCta, GRtx, GRty, GRta, TLta, TCta, TRta, TLtx, TCtx, TRtx, TLty, TCty, TRty;



            autonomousPathUpdate();



            // Telemetry (unchanged)
            panelsTelemetry.debug("Path State", pathState);
            panelsTelemetry.debug("X", String.format("%.2f", follower.getPose().getX()));
            panelsTelemetry.debug("Y", String.format("%.2f", follower.getPose().getY()));
            panelsTelemetry.debug("Heading °", String.format("%.1f", Math.toDegrees(follower.getPose().getHeading())));
            telemetry.addData("far angele", graduatedFarAngle);
            telemetry.addData("L rpm", getRPM(LeftShooter));
            telemetry.addData("R rpm", getRPM(RightShooter));

            panelsTelemetry.update(telemetry);

            telemetry.update();
        }
    }

    public static class Paths {
        public PathChain P1_Forwardaftershots0303;
        public PathChain P2_IntakeRow0303;
        public PathChain P3_TotriShoot066;
        public PathChain P4_IntaheSquare0306;
        public PathChain P5_TotriShoot0909;
        public PathChain ToDetectPoint;
        public PathChain ToNear;
        public PathChain FromNear;
        public PathChain ToFar;
        public PathChain FromFar;
        public PathChain ToMid;
        public PathChain FromMid;
        public PathChain Park;

        public Paths(Follower follower) {
            P1_Forwardaftershots0303 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(88.000, 9.000),
                                    new Pose(89.494, 25.952),
                                    new Pose(85.973, 38.332),
                                    new Pose(94.924, 34.027),
                                    new Pose(102.600, 35.300)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))

                    .build();

            P2_IntakeRow0303 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(102.600, 35.300),

                                    new Pose(127.500, 35.300)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            P3_TotriShoot066 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(127.500, 35.300),

                                    new Pose(92.000, 12.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(20))

                    .build();

            P4_IntaheSquare0306 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(92.000, 12.500),
                                    new Pose(97.000, 16.000),
                                    new Pose(127.000, 8.100)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(20), Math.toRadians(-5))

                    .build();

            P5_TotriShoot0909 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(127.000, 8.100),

                                    new Pose(92.000, 12.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-5), Math.toRadians(25))

                    .build();

            ToDetectPoint = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(92.000, 12.500),

                                    new Pose(110.800, 23.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(25), Math.toRadians(0))

                    .build();

            ToNear = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(110.800, 23.500),
                                    new Pose(110.500, 36.000),
                                    new Pose(118.000, 34.100),
                                    new Pose(126.500, 34.700)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            FromNear = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(126.500, 34.700),

                                    new Pose(92.000, 12.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(25))

                    .build();

            ToFar = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(110.800, 23.500),
                                    new Pose(118.650, 14.900),
                                    new Pose(126.500, 8.300)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            FromFar = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(126.500, 8.300),

                                    new Pose(92.000, 12.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(25))

                    .build();


            ToMid = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(110.800, 23.500),

                                    new Pose(126.500, 23.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            FromMid = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(126.500, 23.500),

                                    new Pose(92.000, 12.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(25))

                    .build();

            Park = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(92.000, 12.500),

                                    new Pose(102.700, 20.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(25), Math.toRadians(25))

                    .build();
        }
    }

//    public static double nearRPM(double distance) {
//        return 0.000106346*Math.pow(distance, 4)  - 0.0173809*Math.pow(distance, 3) + 0.666884*Math.pow(distance, 2) + 23.07246*distance + 1959.92984;
//    }


    public void autonomousPathUpdate() {

        // Graduated information
        Pose currentPose = follower.getPose();
        double robotX = follower.getPose().getX();
        double robotY = follower.getPose().getY();
        double robotHeadingRad = currentPose.getHeading();
        robotHeadingRad = (robotHeadingRad + 2*Math.PI) % (2*Math.PI);
        double robotHeadingDeg = robotHeadingRad * (180/Math.PI);
        double turretX = robotX - (2.5 * Math.cos(robotHeadingRad) );
        double turretY = robotY - (2.5 * Math.sin(robotHeadingRad) );

        double graduatedDistance = Math.sqrt((targetX-turretX)*(targetX-turretX) + (targetY-turretY)*(targetY-turretY));

        double graduatedFarAngle = -0.000000530303*Math.pow(graduatedDistance, 4) + 0.000294697*Math.pow(graduatedDistance, 3) - 0.06123674*Math.pow(graduatedDistance, 2) + 5.643525*graduatedDistance - 194.1875;

        graduatedFarAngle = Math.min(1.0, Math.max(0.45, graduatedFarAngle));


        Angler.setPosition(graduatedFarAngle);


        switch (pathState) {
            case 0:
                runShooterAtRPM(farRPM, currentVoltage);
                if (!follower.isBusy()) {
//                    runShooterAtRPM(farRPM, currentVoltage);

                    if (pathTimer.milliseconds() >= 2500) {
                        shootThreeBallsOn();
                    }
                    if (pathTimer.milliseconds() >= 3200) {
                        shootThreeBallsOff();
                        pathState = 1;
                    }
                }
                break;

            case 1:
                runShooterAtRPM(farRPM, currentVoltage);
                if (!pathStarted) {
                    follower.followPath(paths.P1_Forwardaftershots0303);
                    pathTimer.reset();
                    pathStarted = true;
                }

                if (!follower.isBusy()) {
                    pathStarted = false;
                    pathState = 2;
                }
                break;

            case 2:
                runShooterAtRPM(farRPM, currentVoltage);
                if (!pathStarted) {
                    follower.followPath(paths.P2_IntakeRow0303);
                    pathTimer.reset();
                    pathStarted = true;
                }
                if ((pathTimer.milliseconds() >= 400) && (pathTimer.milliseconds() < 1900)) {
                    Intake.setPower(-1);
                    Transfer.setPower(-1);
                }
                if (pathTimer.milliseconds() >= 1900) {
                    Transfer.setPower(0);
                }
                if ((!follower.isBusy()) && (pathTimer.milliseconds() >= 2300) ){
                    Intake.setPower(0);
                    pathStarted = false;
                    pathState = 3;
                }
                break;


            case 3:
                runShooterAtRPM(farRPM, currentVoltage);
                if (!pathStarted) {
                    follower.followPath(paths.P3_TotriShoot066);
//                    runShooterAtRPM(farRPM, currentVoltage);
                    pathTimer.reset();
                    pathStarted = true;
                }
                if ((!follower.isBusy()) && (pathTimer.milliseconds() >= 2100) ){
                    Intake.setPower(0);
                    pathStarted = false;
                    pathState = 4;
                }

                break;

            case 4:
                runShooterAtRPM(farRPM, currentVoltage);
                if (!pathStarted) {
                    pathTimer.reset();
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
//                    runShooterAtRPM(farRPM, currentVoltage);
                    if (pathTimer.milliseconds() >= 1800) {
                        shootThreeBallsOn();
                    }
                    if (pathTimer.milliseconds() >= 2900) {
                        shootThreeBallsOff();
                        pathStarted = false;
                        pathState = 5;
                    }
                }
                break;


            case 5:
                runShooterAtRPM(farRPM, currentVoltage);
                if (!pathStarted) {
                    follower.followPath(paths.P4_IntaheSquare0306);
                    pathTimer.reset();
                    pathStarted = true;
                }
                if ((pathTimer.milliseconds() >= 400) && (pathTimer.milliseconds() < 1900)) {
                    Intake.setPower(-1);
                    Transfer.setPower(-1);
                }
                if (pathTimer.milliseconds() >= 1900) {
                    Transfer.setPower(0);
                }
                if ((!follower.isBusy()) && (pathTimer.milliseconds() >= 2300) ){
                    Intake.setPower(0);
                    pathStarted = false;
                    pathState = 6;
                }
                break;

            case 6:
                runShooterAtRPM(farRPM, currentVoltage);
                if (!pathStarted) {
                    follower.followPath(paths.P5_TotriShoot0909);
                    pathTimer.reset();
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    pathState = 7;
                }
                break;

            case 7:
                runShooterAtRPM(farRPM, currentVoltage);
                if (!follower.isBusy()) {
//                    runShooterAtRPM(farRPM, currentVoltage);
                    if (pathTimer.milliseconds() >= 1500) {
                        shootThreeBallsOn();
                    }
                    if (pathTimer.milliseconds() >= 2700) {
                        shootThreeBallsOff();
                        pathState = 10;
                    }
                }
                break;

            case 10: //todo detect point
                runShooterAtRPM(farRPM, currentVoltage);
                if (!pathStarted) {
                    follower.followPath(paths.ToDetectPoint);
                    pathTimer.reset();
                    pathStarted = true;
                }

                if (!follower.isBusy()) {

                    //// happens only once


                    //get ta for Purple Left

                    LLResult result;


                    limelight.pipelineSwitch(1);
                    double PLtx, PLty, PLta, PCtx, PCty, PCta, PRtx, PRty, PRta, GLtx, GLty, GLta, GCtx, GCty, GCta, GRtx, GRty, GRta, TLta, TCta, TRta, TLtx, TCtx, TRtx, TLty, TCty, TRty;

                    result = limelight.getLatestResult(); // pipe 1
                    PLtx = result.getTy();
                    PLty = result.getTx();
                    PLta = result.getTa();


                    //get ta for Purple Center
                    limelight.pipelineSwitch(2);
                    sleep(50);

                    result = limelight.getLatestResult(); // pipe 2
                    PCtx = result.getTy();
                    PCty = result.getTx();
                    PCta = result.getTa();


                    //get ta for Purple Right
                    limelight.pipelineSwitch(3);
                    sleep(50);

                    result = limelight.getLatestResult(); // pipe 3
                    PRtx = result.getTy();
                    PRty = result.getTx();
                    PRta = result.getTa();


                    //get ta for Green Left
                    limelight.pipelineSwitch(4);
                    sleep(50);

                    result = limelight.getLatestResult(); // pipe 4
                    GLtx = result.getTy();
                    GLty = result.getTx();
                    GLta = result.getTa();


                    //get ta for Green Center
                    limelight.pipelineSwitch(5);
                    sleep(50);

                    result = limelight.getLatestResult(); // pipe 5
                    GCtx = result.getTy();
                    GCty = result.getTx();
                    GCta = result.getTa();


                    //get ta for Green Center
                    limelight.pipelineSwitch(6);
                    sleep(50);

                    result = limelight.getLatestResult(); // pipe 6
                    GRtx = result.getTy();
                    GRty = result.getTx();
                    GRta = result.getTa();


                    TLtx = PLtx + GLtx;
                    TCtx = PCtx + GCtx;
                    TRtx = PRtx + GRtx;

                    TLty = PLty + GLty;
                    TCty = PCty + GCty;
                    TRty = PRty + GRty;

                    TLta = PLta + GLta;
                    TCta = PCta + GCta;
                    TRta = PRta + GRta;

                    if (TLta >= 2*TRta) { // case: Left much greater than Right (normally no right)
                        if (TCtx >= 0) { ///// TCtx is on the right
                            if (TLta >= TCta) { // case: More on left than in center-right
                                pathState = 11;
                                pathStarted = false;
                            }
                        }

                        else if (TCtx < 0) { ///// TCtx is on the left
                            if (TCtx < -10) { // TCtx is very left
                                pathState = 11;
                                pathStarted = false;
                            }
                            else if (TLtx > -15){ //TLtx is right
                                pathState = 13;
                                pathStarted = false;
                            }
                            else { // Neither: TLtx is far left, and TCtx is center-left
                                if (TLta >= TCta) {
                                    pathState = 11;
                                    pathStarted = false;
                                }
                                else {
                                    pathState = 13;
                                    pathStarted = false;
                                }
                            }
                        }

                    }

                    else if (TRta >= 2*TLta) { // case: Right much greater than Left (normally no left)
                        if (TCtx <= 0) { ///// TCtx is on the left
                            if (TLta >= TCta) { // case: More on right than in center-left
                                pathState = 15;
                                pathStarted = false;
                            }
                        }

                        else if (TCtx > 0) { ///// TCtx is on the right
                            if (TCtx > 10) { // TCtx is very right
                                pathState = 15;
                                pathStarted = false;
                            }
                            else if (TRtx < 15){ //TLtx is left
                                pathState = 13;
                                pathStarted = false;
                            }
                            else { // Neither: TLtx is far right, and TCtx is center-right
                                if (TLta >= TCta) {
                                    pathState = 15;
                                    pathStarted = false;
                                }
                                else {
                                    pathState = 13;
                                    pathStarted = false;
                                }
                            }
                        }

                    }

                    else if ((TCta > TLta) && (TCta > TRta)) { // Center much greater than either side
                        pathState = 13;
                        pathStarted = false;
                    }

                    else if ((TLta > 1.2*TCta) && (TRta > 1.2*TCta)) { // Center does not have balls compared to sides
                        if (TLta > 1.2*TRta) { // Left has more than Right
                            pathState = 11;
                            pathStarted = false;
                        }
                        else if (TRta > 1.2*TLta) { // Right has more than Left
                            pathState = 15;
                            pathStarted = false;
                        }
                        else { // very similar sizes

                        }
                    }

                }
                break;

            case 11:// near
                if (!pathStarted) {
                    follower.followPath(paths.ToNear);
                    pathTimer.reset();
                    pathStarted = true;
                }
                if ((pathTimer.milliseconds() >= 400) && (pathTimer.milliseconds() < 1900)) {
                    Intake.setPower(-1);
                    Transfer.setPower(-1);
                }
                if (pathTimer.milliseconds() >= 1900) {
                    Transfer.setPower(0);
                }
                if ((!follower.isBusy()) && (pathTimer.milliseconds() >= 2300) ){
                    Intake.setPower(0);
                    pathStarted = false;
                    pathState = 12;
                }
                break;

            case 12:
                if (!pathStarted) {
                    follower.followPath(paths.FromNear);
                    runShooterAtRPM(farRPM, currentVoltage);
                    pathTimer.reset();
                    pathStarted = true;
                }
                if (pathTimer.milliseconds() >= 2900) {
                    shootThreeBallsOn();
                }
                if (pathTimer.milliseconds() >= 3800) {
                    shootThreeBallsOff();
                    pathState = 2;
                }
                if ((!follower.isBusy()) && (pathTimer.milliseconds() >= 3800) ){
                    pathStarted = false;
                    pathState = 20;
                }
                break;

            case 13:// mid
                if (!pathStarted) {
                    follower.followPath(paths.ToMid);
                    pathTimer.reset();
                    pathStarted = true;
                }
                if (pathTimer.milliseconds() >= 3900) {
                    shootThreeBallsOn();
                }
                if (pathTimer.milliseconds() >= 5000) {
                    shootThreeBallsOff();
                    pathState = 2;
                }
                if ((!follower.isBusy()) && (pathTimer.milliseconds() >= 5000) ){
                    pathStarted = false;
                    pathState = 14;
                }
                break;

            case 14:
                if (!pathStarted) {
                    follower.followPath(paths.FromMid);
                    runShooterAtRPM(farRPM, currentVoltage);
                    pathTimer.reset();
                    pathStarted = true;
                }
                if (pathTimer.milliseconds() >= 3900) {
                    shootThreeBallsOn();
                }
                if (pathTimer.milliseconds() >= 5000) {
                    shootThreeBallsOff();
                    pathState = 2;
                }
                if ((!follower.isBusy()) && (pathTimer.milliseconds() >= 5000) ){
                    pathStarted = false;
                    pathState = 20;
                }
                break;

            case 15:// far
                if (!pathStarted) {
                    follower.followPath(paths.ToFar);
                    pathTimer.reset();
                    pathStarted = true;
                }
                if ((pathTimer.milliseconds() >= 400) && (pathTimer.milliseconds() < 1900)) {
                    Intake.setPower(-1);
                    Transfer.setPower(-1);
                }
                if (pathTimer.milliseconds() >= 1900) {
                    Transfer.setPower(0);
                }
                if ((!follower.isBusy()) && (pathTimer.milliseconds() >= 2300) ){
                    Intake.setPower(0);
                    pathStarted = false;
                    pathState = 16;
                }
                break;

            case 16:
                if (!pathStarted) {
                    follower.followPath(paths.FromFar);
                    runShooterAtRPM(farRPM, currentVoltage);
                    pathTimer.reset();
                    pathStarted = true;
                }
                if (pathTimer.milliseconds() >= 3900) {
                    shootThreeBallsOn();
                }
                if (pathTimer.milliseconds() >= 5000) {
                    shootThreeBallsOff();
                    pathState = 2;
                }
                if ((!follower.isBusy()) && (pathTimer.milliseconds() >= 5000) ){
                    pathStarted = false;
                    pathState = 20;
                }
                break;

            case 20:
                if (!pathStarted) {
                    follower.followPath(paths.Park);
                    pathTimer.reset();
                    pathStarted = true;
                }

                if (!follower.isBusy()) {
                    pathStarted = false;
                    pathState = 21;
                }
                break;


            case 21:
                if (!follower.isBusy()) {
                    follower.breakFollowing();
                    pathState = 99;
                }
                break;

            case 99:
                requestOpModeStop();
                break;

        }
    }

}