package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.Timing.Timer;

import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OnofreiSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PaleteSubsytem;
import org.firstinspires.ftc.teamcode.subsystems.RoataSubsystem;

public class LaunchBall extends CommandBase {
    private final OnofreiSubsystem onofrei;
    private final PaleteSubsytem palete;
    private final LauncherSubsystem launcher;
    private Timer launchTimer, onofreiTimer;
    private final int sector;
    public LaunchBall(PaleteSubsytem paleteSubsytem, OnofreiSubsystem onofreiSubsystem, LauncherSubsystem launcherSubsystem, int sector) {
        this.palete = paleteSubsytem;
        this.onofrei = onofreiSubsystem;
        this.launcher = launcherSubsystem;
        this.sector = sector;

        addRequirements(palete, onofrei, launcher);
    }

    @Override
    public void initialize() {
        onofreiTimer = new Timer(1000);
        launchTimer = new Timer(1500);
        switch (sector) {
            case 1:
                palete.setPosition(PaleteSubsytem.OUT_BILA_1);
                break;
            case 2:
                palete.setPosition(PaleteSubsytem.OUT_BILA_2);
                break;
            case 3:
                palete.setPosition(PaleteSubsytem.OUT_BILA_3);
                break;
            default:
                throw new IllegalArgumentException("Invalid sector: " + sector);
        }
        launcher.spin();
    }

    @Override
    public void execute() {
        if (onofreiTimer.done()) {
            onofrei.setPosition(OnofreiSubsystem.OUT);
        }
    }

    public boolean isFinished() {
        if (launchTimer.done()) {
            launcher.stop();
            return true;
        } else {
            return false;
        }
    }
}
