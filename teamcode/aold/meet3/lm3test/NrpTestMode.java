//package org.firstinspires.ftc.teamcode.aold.meet3.lm3test;
//
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.hardware.Servo;
//
//
///*
//4 drive motors perfecti sunt
//2 shooting motors(inverses): multiple set powers perfecti est
//1 intake motor: one value setPower(on/off) perfectum est
//
//1 turret servo fit
//1 John Austin Hatch (2 setPosition) pefectum est
// */
//
//public abstract class NrpTestMode extends LinearOpMode {
//
//    public DcMotor RightFront, RightBack, LeftFront, LeftBack, Intake, Transfer;
//    public DcMotorEx LeftShooter, RightShooter;
//    public Servo Turret, TurretR, TurretL, TheJohnAustinHatch, Angler, Lift;
//    public IMU imu;
//    //public BNO055IMU imu;
//    public int LeftFrontPos, LeftBackPos, RightFrontPos, RightBackPos;
//
//    public  Limelight3A limelight;
//    public int turretOscillationDirection, pipelineSelected;
//    public double turretPos, turretTargetX, turretChange;
//
//    // PID constants
//    public static final double TICKS_PER_REV = 28;
//
//
//    //PIDF Konstants
//    public double Kp = 10.8;
//    public double Ki = 0.12;
//    public double Kd = 10.0;
//    public double Kf = 16.1;
//
////    public double Kp = 32.8;
////    public double Ki = 0.12;
////    public double Kd = 5.0;
////    public double Kf = 23.1;
//
//    //RPM targets - Teleop
//    public double FAR_TARGET_RPM = 3055;
//    public double NEAR_TARGET_RPM = 2500;
//    public double NEAR_AUTO_RPM = 2460;
//
//    public double TEST_RPM = 6000;
//
//    public double FAR_TARGET_TICKS_PER_SEC = FAR_TARGET_RPM * TICKS_PER_REV / 60.0;
//    public double NEAR_TARGET_TICKS_PER_SEC = NEAR_TARGET_RPM * TICKS_PER_REV / 60.0;
//    public double TEST_TARGET_TICKS_PER_SEC = TEST_RPM * TICKS_PER_REV / 60.0;
//
//
//
//
//    public void initControls() {
////        RightFront = initMotor("rightFront", false);
////        LeftFront = initMotor("leftFront", true);
////        RightBack = initMotor("rightRear", false);
////        LeftBack = initMotor("leftRear", true);
//        Intake = initMotor("Intake", false);
//        Transfer = initMotor("Transfer", true);
//
//
////        LeftShooter = initMotorEx("LeftShooter", true);
////        RightShooter = initMotorEx("RightShooter", false);
//        // Collector =
////        TurretR = hardwareMap.servo.get("TurretR");
////        TurretL = hardwareMap.servo.get("TurretL");
//
//        TheJohnAustinHatch = hardwareMap.servo.get("TheJohnAustinHatch");
////        Angler = hardwareMap.servo.get("Angler");
////
////        LeftShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        RightShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////
////        LeftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////        RightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
////        imu = hardwareMap.get(IMU.class, "pinpoint");
//
//        // limelight
//
//
//    }
//    public void runShooterAtRPM(double rpm) {
//        double target = rpm * TICKS_PER_REV / 60.0;
//
//        // Apply same settings to both motors
//        for (DcMotorEx motor : new DcMotorEx[]{LeftShooter, RightShooter}) {
//            motor.setVelocity(target);
//            motor.setVelocityPIDFCoefficients(Kp, Ki, Kd, Kf); // P, I, D, F
//        }
//    }
//    public void stopShooter() {
//
//        // Apply same settings to both motors
//        for (DcMotorEx motor : new DcMotorEx[]{LeftShooter, RightShooter}) {
//            motor.setVelocity(0);
//            motor.setVelocityPIDFCoefficients(Kp, Ki, Kd, Kf); // P, I, D, F
//        }
//    }
////    public void shootThreeBalls() {
////        Intake.setPower(0.35);//outtake for a bit
////        sleep(200);
////        TheJohnAustinHatch.setPosition(0.49);//open hatch
////        Intake.setPower(-1);//intake
////        sleep(1750);
//////        Intake.setPower(0.3);//outtake just in case
//////        sleep(100);
//////        Intake.setPower(-1);//intake
//////        sleep(500);
////        TheJohnAustinHatch.setPosition(0.26);//hatch close
////        sleep(500);//finish intake
////        Intake.setPower(0);//stop intake
////
////
////    }
//
////    public void initLimelight(int pipeline) {
////        pipelineSelected = pipeline;
////
////        limelight = hardwareMap.get(Limelight3A.class, "limelight");
////        limelight.pipelineSwitch(pipelineSelected);
////        imu = hardwareMap.get(IMU.class, "imu");
////        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
////                RevHubOrientationOnRobot.UsbFacingDirection.UP);
////        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
////    }
//
//    public DcMotor initMotor(String name, boolean direction) {
//        //direction true=forward false=reverse
//        DcMotor temp = hardwareMap.get(DcMotor.class, name);
//        if (!direction) temp.setDirection(DcMotorSimple.Direction.REVERSE);
//        temp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        return temp;
//    }
//    public DcMotorEx initMotorEx(String name, boolean direction) {
//        //direction true=forward false=reverse
//        DcMotorEx temp = hardwareMap.get(DcMotorEx.class, name);
//        if (!direction) temp.setDirection(DcMotorSimple.Direction.REVERSE);
//        temp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        return temp;
//    }
//    public void initCommands() throws InterruptedException {
//        sleep(0);
//    }
//    //     is busy commands
//    public void driveMotorBusy() {
//        while(LeftBack.isBusy()||RightBack.isBusy()||LeftFront.isBusy()||RightFront.isBusy()){}
////            telemetry.addData("LeftFront",LeftFront.getCurrentPosition());
////            telemetry.addData("LeftBack",LeftBack.getCurrentPosition());
////            telemetry.addData("RightFront",RightFront.getCurrentPosition());
////            telemetry.addData("RightBack",RightBack.getCurrentPosition());
////            telemetry.update();
////
//    }
//    //    public void slideMotorBusy()
////    {
////        while(SlideLeft.isBusy()&&SlideRight.isBusy()){}
////    }
//    //resets and runmodes
//    public void Reset_Encoder() {
//        LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    }
//    //    public void Reset_Encoder_Arm()
////    {
////        SlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        SlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////    }
//    public void Run_Mode_Pos() {
//        LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    }
//    public void Run_Mode_Pow() {
//        LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
//        RightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
//        LeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
//        RightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
//    }
//    //commands for auto
//    public void drive(int LeftFrontTarget, int RightFrontTarget, double speed) {
//        LeftFrontPos += LeftFrontTarget;
//        RightFrontPos += RightFrontTarget;
//
//
//        LeftFront.setTargetPosition(LeftFrontPos);
//        RightFront.setTargetPosition(RightFrontPos);
//
//
//        LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//
//        LeftFront.setPower(speed);
//        RightFront.setPower(speed);
//    }
//    public void driveb(int LeftBackTarget, int RightBackTarget, double speed) {
//        LeftBackPos += LeftBackTarget;
//        RightBackPos += RightBackTarget;
//
//
//        LeftBack.setTargetPosition(LeftBackPos);
//        RightBack.setTargetPosition(RightBackPos);
//
//
//        LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//
//        LeftBack.setPower(speed);
//        RightBack.setPower(speed);
//    }
//    public void all_wheel(int LF,int RF,int LB,int RB,double s) {
//        drive(LF, RF, s);
//        driveb(LB, RB, s);
//    }
////    public void slide_encoder(int ArmTarget, double as)
////    {
////        ArmPos -= ArmTarget;
////        SlideLeftPos += ArmTarget;
////        SlideRightPos += ArmTarget;
////
////
////        SlideLeft.setTargetPosition(ArmPos);
////        SlideRight.setTargetPosition(ArmPos);
////
////
////        SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////
////
////        SlideLeft.setPower(as);
////        SlideRight.setPower(as);
////    }
////    public void all_wheel_pow(double LF,double RF,double LB,double RB)
////    {
////        Run_Mode_Pow();
////        LeftFront.setPower(LF);
////        RightFront.setPower(RF);
////        LeftBack.setPower(LB);
////        RightBack.setPower(RB);
////    }
////    public void AutoCollector(int time)
////    {
////        Intake1.setPower(-1);
////        Intake2.setPower(1);
//////        Intake3.setPower(-1);
//////        Intake4.setPower(1);
////        sleep(time);
////    }
////    public void AutoUnCollector(int time)
////    {
////        Intake1.setPower(1);
////        Intake2.setPower(-1);
//////        Intake3.setPower(1);
//////        Intake4.setPower(-1);
////        sleep(time);
////    }
////    public void AutoKillCollector()
////    {
////        Intake1.setPower(0);
////        Intake2.setPower(0);
//////        Intake3.setPower(0);
//////        Intake4.setPower(0);
////    }
//    //teleop commands
////    public void Collector()
////    {
////        Intake1.setPower(-1);
////        Intake2.setPower(1);
//////        Intake3.setPower(-1);
//////        Intake4.setPower(1);
////    }
////    public void UnCollector()
////    {
////        Intake1.setPower(1);
////        Intake2.setPower(-1);
//////        Intake3.setPower(1);
//////        Intake4.setPower(-1);
////    }
////    public void KillCollector()
////    {
////        Intake1.setPower(0);
////        Intake2.setPower(0);
//////        Intake3.setPower(0);
//////        Intake4.setPower(0);
////    }
//
//
//
//
//
//
//}
//
