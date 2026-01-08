package org.firstinspires.ftc.teamcode.commands;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;

public class UpdatePose extends CommandBase {
    private final TelemetryManager telemetry;
    private final LimelightSubsystem limelight;
    private final Follower follower;

    public UpdatePose(TelemetryManager telemetry, Follower follower, LimelightSubsystem limelightSubsystem) {
        this.limelight = limelightSubsystem;
        this.telemetry = telemetry;
        this.follower = follower;

        addRequirements(limelight);
    }

    @Override
    public void initialize() {
//        limelight.updateRobotHeading(follower.getHeading());
        Pose3D megaTag1Pose = limelight.getMegaTag1Pose();
        if (megaTag1Pose != null) {
            double cordy = -megaTag1Pose.getPosition().x * 39.37007874 + 72;
            double cordx = megaTag1Pose.getPosition().y * 39.37007874 + 72;
            double heading = -megaTag1Pose.getOrientation().getYaw(AngleUnit.RADIANS);
            follower.setPose(new Pose(cordx, cordy, follower.getHeading()));
        } else {
            telemetry.addLine("Relocalization failed: Camera returns null");
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
