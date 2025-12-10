package org.firstinspires.ftc.teamcode.commands;

import com.bylazar.telemetry.TelemetryManager;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;

public class TrackAprilTag extends CommandBase {
    private final TelemetryManager telemetry;
    private final double kP = 0.025, kI = 0, kD = 0.00001, kF = 0;
    private final GamepadEx gamepad;
    private final DriveSubsystem drive;
    private final LimelightSubsystem limelight;
    private final PIDFController controller;

    public TrackAprilTag(GamepadEx gamepad, TelemetryManager telemetry, DriveSubsystem driveSubsystem, LimelightSubsystem limelightSubsystem) {
        this.drive = driveSubsystem;
        this.limelight = limelightSubsystem;
        this.gamepad = gamepad;
        this.telemetry = telemetry;
        this.controller = new PIDFController(kP, kI, kD, kF);
        this.controller.setTolerance(0.25);
        this.controller.setMinimumOutput(0.1);

        addRequirements(drive, limelight); // de testat cum se comporta robotul cand nu dam require la drive
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double output = controller.calculate(-limelight.getTx(), 0);
        if (!controller.atSetPoint()) {
            drive.drive(gamepad.getLeftX() * 1.1, gamepad.getLeftY(), output);
        } else {
            drive.drive(gamepad.getLeftX() * 1.1, gamepad.getLeftY(), gamepad.getRightX());
        }
        telemetry.addData("limelight tx:", limelight.getTx());
        telemetry.addData("power", output);
    }

    @Override
    public void end(boolean interrupted) {

    }

//    @Override
//    public boolean isFinished() {
//        return controller.atSetPoint();
//    }
}
