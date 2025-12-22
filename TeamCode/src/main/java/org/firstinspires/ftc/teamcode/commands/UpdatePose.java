package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;

public class UpdatePose extends CommandBase {
    private final LimelightSubsystem limelight;
    private final Follower follower;

    public UpdatePose(Follower follower, LimelightSubsystem limelightSubsystem) {
        this.limelight = limelightSubsystem;
        this.follower = follower;

        addRequirements(limelight);
    }

    @Override
    public void initialize() {
        Pose3D robotPose3D = limelight.getMegaTagPose();
        follower.setPose(new Pose(robotPose3D.getPosition().x, robotPose3D.getPosition().y,
                robotPose3D.getOrientation().getYaw(AngleUnit.RADIANS)));
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
