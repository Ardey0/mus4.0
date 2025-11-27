package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.Timing.Timer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OnofreiSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PaleteSubsytem;
import org.firstinspires.ftc.teamcode.subsystems.RoataSubsystem;

import java.util.concurrent.TimeUnit;

public class LaunchBall extends CommandBase {
    private final OnofreiSubsystem onofrei;
    private final PaleteSubsytem palete;
    private final LauncherSubsystem launcher;
    private final RoataSubsystem roata;
    private Timer onofreiTimer = new Timer(1000, TimeUnit.MILLISECONDS),
                  paleteTimer = new Timer(1000, TimeUnit.MILLISECONDS),
                  launchTimer = new Timer(1500, TimeUnit.MILLISECONDS);
    private final int motif;
    private String[] order = new String[3];
    private int ball = 0;
    private boolean done = false;

    public LaunchBall(RoataSubsystem roataSubsystem, PaleteSubsytem paleteSubsytem,
                      OnofreiSubsystem onofreiSubsystem, LauncherSubsystem launcherSubsystem,
                      int motif) {
        this.palete = paleteSubsytem;
        this.onofrei = onofreiSubsystem;
        this.launcher = launcherSubsystem;
        this.roata = roataSubsystem;
        this.motif = motif;

        addRequirements(palete, onofrei, launcher);
    }

    @Override
    public void initialize() {
        switch (motif) {
            case 21:
                order[0] = "GREEN";
                order[1] = "PURPLE";
                order[2] = "PURPLE";
                break;
            case 22:
                order[0] = "PURPLE";
                order[1] = "GREEN";
                order[2] = "PURPLE";
                break;
            case 23:
                order[0] = "PURPLE";
                order[1] = "PURPLE";
                order[2] = "GREEN";
                break;
        }
        paleteTimer.start();
        launcher.spin();
    }

    @Override
    public void execute() {
        int sector = roata.getNextSectorWithColor(order[ball]);
        switch (sector) {
//            case -1:
//                done = true;
//                break;
            case 0:
                palete.setPosition(PaleteSubsytem.OUT_BILA_1);
                if (onofreiTimer.remainingTime() > 0) {
                    onofrei.setPosition(OnofreiSubsystem.IN);
                    paleteTimer.start();
                    ball++;
                }
                if (paleteTimer.remainingTime() <= 0) {
                    onofrei.setPosition(OnofreiSubsystem.OUT);
                    onofreiTimer.start();
                }
                break;
            case 1:
                palete.setPosition(PaleteSubsytem.OUT_BILA_2);
                if (onofreiTimer.remainingTime() > 0) {
                    onofrei.setPosition(OnofreiSubsystem.IN);
                    paleteTimer.start();
                    ball++;
                }
                if (paleteTimer.remainingTime() > 0) {
                    onofrei.setPosition(OnofreiSubsystem.OUT);
                    onofreiTimer.start();
                }
                break;
            case 2:
                palete.setPosition(PaleteSubsytem.OUT_BILA_3);
                if (onofreiTimer.remainingTime() > 0) {
                    onofrei.setPosition(OnofreiSubsystem.IN);
                    paleteTimer.start();
                    ball++;
                }
                if (paleteTimer.remainingTime() > 0) {
                    onofrei.setPosition(OnofreiSubsystem.OUT);
                    onofreiTimer.start();
                }
                break;
            default:
                throw new IllegalArgumentException("esti prea prost " + sector);
        }
    }

    @Override
    public void end(boolean interrupted) {
        launcher.stop();
        onofrei.setPosition(OnofreiSubsystem.IN);
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
