package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.OnofreiSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PaleteSubsytem;

public class Init extends CommandBase {
    private final PaleteSubsytem palete;
    private final OnofreiSubsystem onofrei;

    public Init(PaleteSubsytem paleteSubsytem, OnofreiSubsystem onofreiSubsystem) {
        this.palete = paleteSubsytem;
        this.onofrei = onofreiSubsystem;
    }

    @Override
    public void initialize() {
        onofrei.setPosition(OnofreiSubsystem.IN);
        palete.setPosition(PaleteSubsytem.LOCK);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
