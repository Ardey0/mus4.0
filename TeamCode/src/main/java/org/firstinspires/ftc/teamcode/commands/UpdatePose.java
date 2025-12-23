package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;

public class UpdatePose extends CommandBase {
    private final LimelightSubsystem limelight;
    private final Follower follower;
    private final GoBildaPinpointDriver pinpoint;

    public UpdatePose(Follower follower, GoBildaPinpointDriver pinpoint, LimelightSubsystem limelightSubsystem) {
        this.limelight = limelightSubsystem;
        this.follower = follower;
        this.pinpoint = pinpoint;

        addRequirements(limelight);
    }

    @Override
    public void initialize() {
        limelight.updateRobotHeading(pinpoint.getHeading(AngleUnit.DEGREES));
        Pose3D robotPose3D = limelight.getMegaTagPose();
        follower.setPose(new Pose(robotPose3D.getPosition().x, robotPose3D.getPosition().y,
                robotPose3D.getOrientation().getYaw(AngleUnit.RADIANS)));
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
