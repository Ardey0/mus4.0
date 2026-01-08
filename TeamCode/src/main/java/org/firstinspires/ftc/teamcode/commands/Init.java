package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeKickerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OnofreiSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PaleteSubsytem;
import org.firstinspires.ftc.teamcode.subsystems.RampaSubsystem;

public class Init extends CommandBase {
    private final PaleteSubsytem palete;
    private final OnofreiSubsystem onofrei;
    private final RampaSubsystem rampa;
    private final IntakeKickerSubsystem intakeKicker;

    public Init(PaleteSubsytem paleteSubsytem, OnofreiSubsystem onofreiSubsystem, RampaSubsystem rampaSubsystem, IntakeKickerSubsystem intakeKickerSubsystem) {
        this.palete = paleteSubsytem;
        this.onofrei = onofreiSubsystem;
        this.rampa = rampaSubsystem;
        this.intakeKicker = intakeKickerSubsystem;
    }

    @Override
    public void initialize() {
        onofrei.setPosition(OnofreiSubsystem.IN);
        palete.setPosition(PaleteSubsytem.LOCK);
        rampa.setPosition(RampaSubsystem.JOS);
        intakeKicker.setPosition(IntakeKickerSubsystem.IN);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
