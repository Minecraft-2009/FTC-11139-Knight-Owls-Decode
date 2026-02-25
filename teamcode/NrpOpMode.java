package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;



/*
4 drive motors perfecti sunt
2 shooting motors(inverses): multiple set powers perfecti est
1 intake motor: one value setPower(on/off) perfectum est

1 turret servo fit
1 John Austin Hatch (2 setPosition) pefectum est
 */

public abstract class NrpOpMode extends LinearOpMode {

    public DcMotor RightFront, RightBack, LeftFront, LeftBack, Intake, Transfer;
    public DcMotorEx LeftShooter, RightShooter;
    public Servo Turret, TurretR, TurretL, TheJohnAustinHatch, Angler, Lift, RGBlight;
    public IMU imu;
    public NormalizedColorSensor RightFrontCS, RightMiddleCS, LeftFrontCS, LeftMiddleCS, TopCS;
    //public BNO055IMU imu;
    public int LeftFrontPos, LeftBackPos, RightFrontPos, RightBackPos;

    public  Limelight3A limelight;
    public int turretOscillationDirection, pipelineSelected;
    public double turretPos, turretTargetX, turretChange;
    public PIDFControllerNRP controller = new PIDFControllerNRP();
    public double tunedVoltage = 12.15;

    // PID constants
    // CHANGE THESE THREE LINES ONLY
    public static final double TICKS_PER_REV = 28;
    public VoltageSensor batteryVoltageSensor;


    //PIDF CONSTANTS - TUNING
//    public double Kp = 2.65;
//    public double Ki = 0.00;
//    public double Kd = 0.02;
//    public double Kf = 12.44;


    // DYNAMIC PIDF
    public double Kp1 = 0.00; // Near =
    public double Ki1 = 0.00;
    public double Kd1 = 0.00;
    public double Kf1 = 11.7;

    public double P = 0.002; //TODO: these ones are ued
    public double I = 0.00;
    public double kS = 0.0004;
    public double kV = 0.00041975;

//    public double Kp2 = 2.19; // Mid =
//    public double Ki2 = 0.05;
//    public double Ki22 = 0.0;
//    public double Kd2 = 0.015;
//    public double Kf2 = 12.21;
//
//    public double Kp3 = 0.01; //Far =
//    public double Ki3 = 0.055;
//    public double Ki32 = 0.00;
//    public double Kd3 = 0.015;
//    public double Kf3 = 11.50;

//    public double Kp = 14.0;
//    public double Ki = 0.02;
//    public double Kd = 0.0;
//    public double Kf = 13.55;


    //RPM TARGETS TELEOP
    public double FAR_TARGET_RPM = 4600; //TODO:
    public double NEAR_TARGET_RPM = 3500; //TODO:
    public double NEAR_AUTO_RPM = 3460;

    public double farRPM = 4500;


    public double FAR_TARGET_TICKS_PER_SEC = FAR_TARGET_RPM * TICKS_PER_REV / 60.0;
    public double NEAR_TARGET_TICKS_PER_SEC = NEAR_TARGET_RPM * TICKS_PER_REV / 60.0;




    public void initControls() {
        RightFront = initMotor("rightFront", true);
        LeftFront = initMotor("leftFront", false);
        RightBack = initMotor("rightRear", true);
        LeftBack = initMotor("leftRear", false);
        Intake = initMotor("Intake", false);
        Transfer = initNoBrakeMotor("Transfer", true);

        LeftShooter = initMotorEx("LeftShooter", true);
        RightShooter = initMotorEx("RightShooter", false);

        TheJohnAustinHatch = hardwareMap.servo.get("TheJohnAustinHatch");
        Angler = hardwareMap.servo.get("Angler");
        TurretR = hardwareMap.servo.get("TurretR");
        TurretL = hardwareMap.servo.get("TurretL");
        Lift = hardwareMap.servo.get("Lift");
        RGBlight = hardwareMap.servo.get("RGBlight");

        LeftShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RightFrontCS = hardwareMap.get(NormalizedColorSensor.class, "RightFrontCS");
        RightMiddleCS = hardwareMap.get(NormalizedColorSensor.class, "RightMiddleCS");
        LeftFrontCS = hardwareMap.get(NormalizedColorSensor.class, "LeftFrontCS");
        LeftMiddleCS = hardwareMap.get(NormalizedColorSensor.class, "LeftMiddleCS");
        TopCS = hardwareMap.get(NormalizedColorSensor.class, "TopCS");






        // limelight

    }
    public void initVoltageSensor(){
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
    }


    public void runShooterAtRPM(double rpm, double currentVoltage) {
        double target = rpm * TICKS_PER_REV / 60.0;

        controller.setPIDF(P, I, 0.0, kV * target + kS);
        double curVelocity = LeftShooter.getVelocity();
        double error = target-curVelocity;

        double feedforward = kS + kV*target;
        double feedback = error*P;
        double batteryCompensation = tunedVoltage/currentVoltage;

//              ////  targetPower = Math.max(0, Math.min(1, -controller.calculate(testRPM - curVelocity)));
        double targetPower = Math.max(0, Math.min(1, (feedback+feedforward)*batteryCompensation));
        LeftShooter.setPower(targetPower);
        RightShooter.setPower(targetPower);
    }
    public void runShooterAtRPM(double rpm) {
        double target = rpm * TICKS_PER_REV / 60.0;
        LeftShooter.setVelocity(target);
        LeftShooter.setVelocityPIDFCoefficients(Kp1, Ki1, Kd1, Kf1); // P, I, D, F
        RightShooter.setVelocity(target);
        RightShooter.setVelocityPIDFCoefficients(Kp1, Ki1, Kd1, Kf1); // P, I, D, F
    }

    public void runPIDFShooterAtRPM(double rpm, double kp, double ki, double kd, double kf) {
        double target = rpm * TICKS_PER_REV / 60.0;


//        if (rpm < 2500) {
        LeftShooter.setVelocity(target);
        LeftShooter.setVelocityPIDFCoefficients(kp, ki, kd, kf); // P, I, D, F
        RightShooter.setVelocity(target);
        RightShooter.setVelocityPIDFCoefficients(kp, ki, kd, kf); // P, I, D, F
    }

    public double getRPM(DcMotorEx motorEx) {
        return motorEx.getVelocity() * 60 / TICKS_PER_REV;
    }



    public void stopShooter() {

        LeftShooter.setMotorDisable();
        RightShooter.setMotorDisable();
    }
    public void shootThreeBalls() {
        TheJohnAustinHatch.setPosition(0.25);//open hatch
        Intake.setPower(-1);//intake
        Transfer.setPower(-1);
        sleep(500);
        Intake.setPower(0);//stop intake
        Transfer.setPower(0);
    }

    public void shootThreeBallsOn() {
        TheJohnAustinHatch.setPosition(0.25);//open hatch
        sleep(20);
        Intake.setPower(-1);
        Transfer.setPower(-1);
    }
    public void shootThreeBallsOff() {
        Intake.setPower(0);
        Transfer.setPower(0);
        TheJohnAustinHatch.setPosition(0);//close hatch
    }

//    public void initLimelight(int pipeline) {
//        pipelineSelected = pipeline;
//
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        limelight.pipelineSwitch(pipelineSelected);
//        imu = hardwareMap.get(IMU.class, "imu");
//        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
//                RevHubOrientationOnRobot.UsbFacingDirection.UP);
//        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
//    }

    public DcMotor initMotor(String name, boolean direction) {
        //direction true=forward false=reverse
        DcMotor temp = hardwareMap.get(DcMotor.class, name);
        if (!direction) temp.setDirection(DcMotorSimple.Direction.REVERSE);
        temp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return temp;
    }
    public DcMotor initNoBrakeMotor(String name, boolean direction) {
        //direction true=forward false=reverse
        DcMotor temp = hardwareMap.get(DcMotor.class, name);
        if (!direction) temp.setDirection(DcMotorSimple.Direction.REVERSE);
        return temp;
    }
    public DcMotorEx initMotorEx(String name, boolean direction) {
        //direction true=forward false=reverse
        DcMotorEx temp = hardwareMap.get(DcMotorEx.class, name);
        if (!direction) temp.setDirection(DcMotorSimple.Direction.REVERSE);
        return temp;
    }
    public void initCommands() throws InterruptedException {
        sleep(0);
    }
    //     is busy commands
    public void driveMotorBusy() {
        while(LeftBack.isBusy()||RightBack.isBusy()||LeftFront.isBusy()||RightFront.isBusy()){}
//            telemetry.addData("LeftFront",LeftFront.getCurrentPosition());
//            telemetry.addData("LeftBack",LeftBack.getCurrentPosition());
//            telemetry.addData("RightFront",RightFront.getCurrentPosition());
//            telemetry.addData("RightBack",RightBack.getCurrentPosition());
//            telemetry.update();
//
    }
    //    public void slideMotorBusy()
//    {
//        while(SlideLeft.isBusy()&&SlideRight.isBusy()){}
//    }
    //resets and runmodes
    public void Reset_Encoder() {
        LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    //    public void Reset_Encoder_Arm()
//    {
//        SlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        SlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    }
    public void Run_Mode_Pos() {
        LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void Run_Mode_Pow() {
        LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        RightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        LeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        RightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
    }
    //commands for auto
    public void drive(int LeftFrontTarget, int RightFrontTarget, double speed) {
        LeftFrontPos += LeftFrontTarget;
        RightFrontPos += RightFrontTarget;

        LeftFront.setTargetPosition(LeftFrontPos);
        RightFront.setTargetPosition(RightFrontPos);


        LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        LeftFront.setPower(speed);
        RightFront.setPower(speed);
    }
    public void driveb(int LeftBackTarget, int RightBackTarget, double speed) {
        LeftBackPos += LeftBackTarget;
        RightBackPos += RightBackTarget;


        LeftBack.setTargetPosition(LeftBackPos);
        RightBack.setTargetPosition(RightBackPos);


        LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        LeftBack.setPower(speed);
        RightBack.setPower(speed);
    }
    public void all_wheel(int LF,int RF,int LB,int RB,double s) {
        drive(LF, RF, s);
        driveb(LB, RB, s);
    }
//    public void slide_encoder(int ArmTarget, double as)
//    {
//        ArmPos -= ArmTarget;
//        SlideLeftPos += ArmTarget;
//        SlideRightPos += ArmTarget;
//
//
//        SlideLeft.setTargetPosition(ArmPos);
//        SlideRight.setTargetPosition(ArmPos);
//
//
//        SlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        SlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//
//        SlideLeft.setPower(as);
//        SlideRight.setPower(as);
//    }
//    public void all_wheel_pow(double LF,double RF,double LB,double RB)
//    {
//        Run_Mode_Pow();
//        LeftFront.setPower(LF);
//        RightFront.setPower(RF);
//        LeftBack.setPower(LB);
//        RightBack.setPower(RB);
//    }
//    public void AutoCollector(int time)
//    {
//        Intake1.setPower(-1);
//        Intake2.setPower(1);
////        Intake3.setPower(-1);
////        Intake4.setPower(1);
//        sleep(time);
//    }
//    public void AutoUnCollector(int time)
//    {
//        Intake1.setPower(1);
//        Intake2.setPower(-1);
////        Intake3.setPower(1);
////        Intake4.setPower(-1);
//        sleep(time);
//    }
//    public void AutoKillCollector()
//    {
//        Intake1.setPower(0);
//        Intake2.setPower(0);
////        Intake3.setPower(0);
////        Intake4.setPower(0);
//    }
    //teleop commands
//    public void Collector()
//    {
//        Intake1.setPower(-1);
//        Intake2.setPower(1);
////        Intake3.setPower(-1);
////        Intake4.setPower(1);
//    }
//    public void UnCollector()
//    {
//        Intake1.setPower(1);
//        Intake2.setPower(-1);
////        Intake3.setPower(1);
////        Intake4.setPower(-1);
//    }
//    public void KillCollector()
//    {
//        Intake1.setPower(0);
//        Intake2.setPower(0);
////        Intake3.setPower(0);
////        Intake4.setPower(0);
//    }






}

