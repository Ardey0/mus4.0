package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;

public class ReadMotif extends CommandBase {
    private final LimelightSubsystem limelight;
    public int tagId = 0;

    public ReadMotif(LimelightSubsystem limelightSubsystem) {
        this.limelight = limelightSubsystem;
    }

    @Override
    public void initialize() {
        tagId = limelight.aprilTagId();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
