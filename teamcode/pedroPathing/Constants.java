package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13.2)
            .forwardZeroPowerAcceleration(-31.490104957828997)
            .lateralZeroPowerAcceleration(-57.49515934512814)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.042, 0, 0.0, 0))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.290,0,0.01,0.015));
//            .headingPIDFCoefficients(new PIDFCoefficients(0.12, 0.0, 0.008, 0.0))
//            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.16, 0.0, 0.012, 0.0))
//            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.1, 0.0, 0.01, 0.65, 0.0))
//            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.1, 0.0, 0.01, 0.65, 0.0))
//            .centripetalScaling(0.005);


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(0.9)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightRear")
            .leftRearMotorName("leftRear")
            .leftFrontMotorName("leftFront")
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(62.833321158341526)
            .yVelocity(69.420);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(1.5)
            .strafePodX(-7.0)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
