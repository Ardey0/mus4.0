package org.firstinspires.ftc.teamcode.commands;

import com.bylazar.telemetry.TelemetryManager;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotStorage;

public class ReadMotif extends CommandBase {
    private final LimelightSubsystem limelight;
    private final RobotStorage robotStorage;
    private final TelemetryManager telemetry;
    public int tagId = 0;

    public ReadMotif(RobotStorage robotStorage, TelemetryManager panelsTelemetry, LimelightSubsystem limelightSubsystem) {
        this.limelight = limelightSubsystem;
        this.robotStorage = robotStorage;
        this.telemetry = panelsTelemetry;
    }

    @Override
    public void initialize() {
        tagId = limelight.getAprilTagId();
        robotStorage.setMotif(tagId);
        telemetry.addData("tag id:", tagId);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
