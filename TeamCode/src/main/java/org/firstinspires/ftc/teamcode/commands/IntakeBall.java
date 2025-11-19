package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class IntakeBall extends CommandBase {
    private final IntakeSubsystem intake;

    public IntakeBall(IntakeSubsystem intakeSubsystem) {
        this.intake = intakeSubsystem;

        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.suck();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }

    @Override
    public boolean isFinished() {
        return intake.getDistanceMM() < 10;
    }
}
