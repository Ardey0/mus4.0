package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.subsystems.IntakeKickerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OnofreiSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PaleteSubsytem;
import org.firstinspires.ftc.teamcode.subsystems.RampaSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TiltSubsystem;

import java.util.concurrent.TimeUnit;

public class Init extends CommandBase {
    private final PaleteSubsytem palete;
    private final OnofreiSubsystem onofrei;
    private final RampaSubsystem rampa;
    private final IntakeKickerSubsystem intakeKicker;
    private final TiltSubsystem tilt;
    private final Timing.Timer tiltTimer = new Timing.Timer(100, TimeUnit.MILLISECONDS);

    public Init(PaleteSubsytem paleteSubsytem, OnofreiSubsystem onofreiSubsystem, RampaSubsystem rampaSubsystem,
                IntakeKickerSubsystem intakeKickerSubsystem, TiltSubsystem tiltSubsystem) {
        this.palete = paleteSubsytem;
        this.onofrei = onofreiSubsystem;
        this.rampa = rampaSubsystem;
        this.intakeKicker = intakeKickerSubsystem;
        this.tilt = tiltSubsystem;

        addRequirements(palete, onofrei, rampa, intakeKicker);
    }

    @Override
    public void initialize() {
        tilt.setPosition(0.1);
        onofrei.setPosition(OnofreiSubsystem.IN);
        palete.setPosition(PaleteSubsytem.LOCK);
        rampa.setPosition(RampaSubsystem.IN);
        intakeKicker.setPosition(IntakeKickerSubsystem.IN);
        tiltTimer.start();
    }

    @Override
    public void execute() {
        if (tiltTimer.done()) {
            tilt.setPosition(TiltSubsystem.DOWN);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
