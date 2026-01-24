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
            .forwardZeroPowerAcceleration(-41.70160136729648)
            .lateralZeroPowerAcceleration(-74.33111962062584)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.073, 0, 0.0057, 0.02))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.19, 0, 0.01, 0.015))
            .headingPIDFCoefficients(new PIDFCoefficients(0.8, 0, 0.03, 0.022))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(2,0,0.012,0.017))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.05, 0, 0.0012, 0.6, 0.11))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.014, 0, 0.0023, 0.6, 0.01))
            .centripetalScaling(0.0006);

//            .translationalPIDFCoefficients(new PIDFCoefficients(0.095, 0, 0.0135, 0.022));
//            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.19, 0, 0.019, 0.014))
//            .headingPIDFCoefficients(new PIDFCoefficients(1.2, 0, 0.019, 0.025))
//            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1.72,0,0.01,0.032))
//            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.0078, 0, 0.0004, 0.6, 0.11))
//            .centripetalScaling(0.0004);


    public static MecanumConstants driveConstants = new MecanumConstants()
//            .maxPower(0.3)
            .rightFrontMotorName("front_right")
            .rightRearMotorName("back_right")
            .leftRearMotorName("back_left")
            .leftFrontMotorName("front_left")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(86.4283264640748)
            .yVelocity(65.9201650544144);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-55.9)
            .strafePodX(30.334)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(0.995,
            0.1,
            0.1,
            0.007,
            150,
            1.5,
            10,
            1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .pinpointLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
