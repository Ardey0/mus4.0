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
            .forwardZeroPowerAcceleration(-36.21)
            .lateralZeroPowerAcceleration(-77.16)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(false)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.072, 0, 0.006, 0.02))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.17, 0, 0.01, 0.019))
            .headingPIDFCoefficients(new PIDFCoefficients(0.8, 0, 0.026, 0.018))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1.4,0,0.007,0.0084))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.0033, 0, 0.00035, 0.6, 0.09))
            .centripetalScaling(0.0000000001);

//            .translationalPIDFCoefficients(new PIDFCoefficients(0.095, 0, 0.0135, 0.022));
//            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.19, 0, 0.019, 0.014))
//            .headingPIDFCoefficients(new PIDFCoefficients(1.2, 0, 0.019, 0.025))
//            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1.72,0,0.01,0.032))
//            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.0078, 0, 0.0004, 0.6, 0.11))
//            .centripetalScaling(0.0004);


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
            .xVelocity(90.07)
            .yVelocity(67.53);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-42)
            .strafePodX(34)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
//            .customEncoderResolution(4096 / (35 * Math.PI))
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(0.995,
            0.1,
            0.1,
            0.007,
            150,
            1.4,
            10,
            0.8);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .pinpointLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
