package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class SpitBalls extends CommandBase {
    private final IntakeSubsystem intake;

    public SpitBalls(IntakeSubsystem intakeSubsystem) {
        this.intake = intakeSubsystem;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.spit(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}
