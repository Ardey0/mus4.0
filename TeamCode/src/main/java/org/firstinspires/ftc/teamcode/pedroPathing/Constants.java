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
            .mass(14)
            .forwardZeroPowerAcceleration(-37.3)
            .lateralZeroPowerAcceleration(-77.38)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryDrivePIDF(false)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.095, 0, 0.017, 0.025)) // i = 0.06
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.14, 0, 0.004, 0.015))
            .headingPIDFCoefficients(new PIDFCoefficients(1.41, 0, 0.019, 0.024))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.0075, 0, 0.0016, 0.6, 0.1))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.009, 0, 0.0017, 0.6, 0.09))
            .centripetalScaling(0.0006);


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("front_right")
            .rightRearMotorName("back_right")
            .leftRearMotorName("back_left")
            .leftFrontMotorName("front_left")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(82.35)
            .yVelocity(65.93);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(104)
            .strafePodX(27)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName("pinpoint")
            .customEncoderResolution(4090/(35*Math.PI))
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(0.99,
            100,
            1.4,
            1.2);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .pinpointLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .build();
    }


}
