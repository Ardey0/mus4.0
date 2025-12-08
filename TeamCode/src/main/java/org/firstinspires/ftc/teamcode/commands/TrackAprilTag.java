package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;

public class TrackAprilTag extends CommandBase {
    private final double kP = 0, kI = 0, kD = 0, kF = 0, kS = 0;
    private final GamepadEx gamepad;
    private final DriveSubsystem drive;
    private final LimelightSubsystem limelight;
    private final PIDFController controller = new PIDFController(kP, kI, kD, kF);

    public TrackAprilTag(DriveSubsystem driveSubsystem, LimelightSubsystem limelightSubsystem, GamepadEx gamepad) {
        this.drive = driveSubsystem;
        this.limelight = limelightSubsystem;
        this.gamepad = gamepad;

        addRequirements(limelight); // de testat cum se comporta robotul cand nu dam require la drive
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double output = controller.calculate(limelight.getTx(), 0);
        output = output + kS * Math.signum(output);
        drive.drive(gamepad.getLeftX() * 1.1, gamepad.getLeftY(), output);
    }

    @Override
    public void end(boolean interrupted) {

    }
}
