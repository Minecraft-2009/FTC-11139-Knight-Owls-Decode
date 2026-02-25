package org.firstinspires.ftc.teamcode.testing;



import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.NrpOpMode;
//import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "test the lift + drive")

public class Test extends NrpOpMode {

    public static Follower follower;




    double regularDivBy = 1;

    public void runOpMode() {
        //init stuff
        initControls();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, Math.toRadians(90)));
        follower.update();

        follower.startTeleopDrive();
        follower.update();

        try {
            initCommands();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        waitForStart();

//
        if (opModeIsActive()) {

//            // Wheel code
            while (opModeIsActive()) {

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
//////                    telemetry.addData("LB", v1 * regularDivBy);
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

                follower.setTeleOpDrive(-gamepad2.left_stick_y, -gamepad2.left_stick_x, -gamepad2.right_stick_x, true);
                follower.update();

                telemetry.addData("x:", follower.getPose().getX());
                telemetry.addData("y:" , follower.getPose().getY());
                telemetry.addData("heading:" , follower.getPose().getHeading());
                telemetry.addData("total heading:" , follower.getTotalHeading());

//
//                if (gamepad2.dpad_up) {
//                    LeftBack.setPower(1);
//                    RightBack.setPower(1);
//                    LeftFront.setPower(1);
//                    RightFront.setPower(1);
//                } else if (gamepad2.dpad_left) {
//                    LeftBack.setPower(1);
//                    RightBack.setPower(-1);
//                    LeftFront.setPower(-1);
//                    RightFront.setPower(1);
//                } else if (gamepad2.dpad_right) {
//                    LeftBack.setPower(-1);
//                    RightBack.setPower(1);
//                    LeftFront.setPower(1);
//                    RightFront.setPower(-1);
//                } else if (gamepad2.dpad_down) {
//                    LeftBack.setPower(-1);
//                    RightBack.setPower(-1);
//                    LeftFront.setPower(-1);
//                    RightFront.setPower(-1);
//                } else {
//                    LeftFront.setPower(0);
//                    LeftBack.setPower(0);
//                    RightFront.setPower(0);
//                    RightBack.setPower(0);
//                }

//
                if (gamepad2.yWasPressed()) {
                    Lift.setPosition(0.96); //Out of the way
                } else if (gamepad2.xWasPressed()) {
                    Lift.setPosition(0.875); // Defense mode
                }


                telemetry.update();
            }

        }
    }
}

