package org.firstinspires.ftc.teamcode.commands;

import com.bylazar.telemetry.PanelsTelemetry;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.Timing.Timer;

import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OnofreiSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PaleteSubsytem;
import org.firstinspires.ftc.teamcode.subsystems.RobotStorage;

import java.util.concurrent.TimeUnit;

public class LaunchBall extends CommandBase {
    private final OnofreiSubsystem onofrei;
    private final PaleteSubsytem palete;
    private final LauncherSubsystem launcher;
    private final RobotStorage robotStorage;
    private final PanelsTelemetry telemetry;
    private Timer onofreiTimer = new Timer(1000, TimeUnit.MILLISECONDS),
                  paleteTimer = new Timer(1000, TimeUnit.MILLISECONDS),
                  launchTimer = new Timer(1500, TimeUnit.MILLISECONDS);
    private int ball = 0;
    private boolean done = false;

    public LaunchBall(RobotStorage robotStorage, PanelsTelemetry telemetry, PaleteSubsytem paleteSubsytem,
                      OnofreiSubsystem onofreiSubsystem, LauncherSubsystem launcherSubsystem) {
        this.palete = paleteSubsytem;
        this.onofrei = onofreiSubsystem;
        this.launcher = launcherSubsystem;
        this.robotStorage = robotStorage;
        this.telemetry = telemetry;

        addRequirements(palete, onofrei, launcher);
    }

    @Override
    public void initialize() {
        paleteTimer.start();
        launcher.spin();
    }

    @Override
    public void execute() {
        int sector = robotStorage.getNextSectorWithMotifBall(ball);
        switch (sector) {
            case -1:
                done = true;
                break;
            case 0:
                palete.setPosition(PaleteSubsytem.OUT_BILA_1);
                if (onofreiTimer.isTimerOn() && onofreiTimer.remainingTime() <= 0) {
                    onofrei.setPosition(OnofreiSubsystem.IN);
                    paleteTimer.start();
                    onofreiTimer.pause();
                    ball++;
                }
                if (paleteTimer.isTimerOn() && paleteTimer.remainingTime() <= 0) {
                    onofrei.setPosition(OnofreiSubsystem.OUT);
                    onofreiTimer.start();
                    paleteTimer.pause();
                }
                break;
            case 1:
                palete.setPosition(PaleteSubsytem.OUT_BILA_2);
                if (onofreiTimer.isTimerOn() && onofreiTimer.remainingTime() <= 0) {
                    onofrei.setPosition(OnofreiSubsystem.IN);
                    paleteTimer.start();
                    onofreiTimer.pause();
                    ball++;
                }
                if (paleteTimer.isTimerOn() && paleteTimer.remainingTime() <= 0) {
                    onofrei.setPosition(OnofreiSubsystem.OUT);
                    onofreiTimer.start();
                    paleteTimer.pause();
                }
                break;
            case 2:
                palete.setPosition(PaleteSubsytem.OUT_BILA_3);
                if (onofreiTimer.isTimerOn() && onofreiTimer.remainingTime() <= 0) {
                    onofrei.setPosition(OnofreiSubsystem.IN);
                    paleteTimer.start();
                    onofreiTimer.pause();
                    ball++;
                }
                if (paleteTimer.isTimerOn() && paleteTimer.remainingTime() <= 0) {
                    onofrei.setPosition(OnofreiSubsystem.OUT);
                    onofreiTimer.start();
                    paleteTimer.pause();
                }
                break;
            default:
                throw new IllegalArgumentException("esti prea prost " + sector);
        }
        telemetry.getTelemetry().addData("sector:", sector);
        telemetry.getTelemetry().addData("ball:", ball);
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
