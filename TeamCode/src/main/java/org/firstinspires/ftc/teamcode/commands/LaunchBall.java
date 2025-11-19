package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.Timing.Timer;

import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RoataSubsystem;

public class LaunchBall extends CommandBase {
    private final RoataSubsystem roata;
    private final LauncherSubsystem launcher;
    private Timer launchTimer, onofreiTimer;
    private final int sector;
    public LaunchBall(RoataSubsystem roataSubsystem, LauncherSubsystem launcherSubsystem, int sector) {
        this.roata = roataSubsystem;
        this.launcher = launcherSubsystem;
        this.sector = sector;

        addRequirements(roata, launcher);
    }

    @Override
    public void initialize() {
        onofreiTimer = new Timer(1000);
        launchTimer = new Timer(1000);
        switch (sector) {
            case 1:
                roata.setPaletePosition(0.85);
                break;
            case 2:
                roata.setPaletePosition(0.14);
                break;
            case 3:
                roata.setPaletePosition(0.5);
                break;
            default:
                throw new IllegalArgumentException("Invalid sector: " + sector);
        }
        launcher.spin();
    }

    @Override
    public void execute() {
        if (onofreiTimer.done()) {
            roata.setOnofreiPosition(1);
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
