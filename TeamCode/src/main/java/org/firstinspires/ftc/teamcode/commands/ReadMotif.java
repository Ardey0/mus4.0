package org.firstinspires.ftc.teamcode.commands;

import com.bylazar.telemetry.PanelsTelemetry;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotStorage;

public class ReadMotif extends CommandBase {
    private final LimelightSubsystem limelight;
    private final RobotStorage robotStorage;
    private final PanelsTelemetry telemetry;
    public int tagId = 0;

    public ReadMotif(RobotStorage robotStorage, PanelsTelemetry panelsTelemetry, LimelightSubsystem limelightSubsystem) {
        this.limelight = limelightSubsystem;
        this.robotStorage = robotStorage;
        this.telemetry = panelsTelemetry;
    }

    @Override
    public void initialize() {
        tagId = limelight.aprilTagId();
        robotStorage.setMotif(tagId);
        telemetry.getTelemetry().addData("tag id:", tagId);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
