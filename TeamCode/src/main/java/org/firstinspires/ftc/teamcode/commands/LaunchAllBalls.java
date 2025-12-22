package org.firstinspires.ftc.teamcode.commands;

import com.bylazar.telemetry.TelemetryManager;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.Timing.Timer;

import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OnofreiSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PaleteSubsytem;
import org.firstinspires.ftc.teamcode.subsystems.RobotStorage;

import java.util.concurrent.TimeUnit;

public class LaunchAllBalls extends CommandBase {
    private final OnofreiSubsystem onofrei;
    private final PaleteSubsytem palete;
    private final LauncherSubsystem launcher;
    private final RobotStorage robotStorage;
    private final LimelightSubsystem limelight;
    private final TelemetryManager telemetry;
    private final Timer onofreiTimer = new Timer(400, TimeUnit.MILLISECONDS);
    private final Timer paleteTimer = new Timer(500, TimeUnit.MILLISECONDS);
    private boolean done = false, start = false;
    private double launcherSpeed;

    private enum LaunchStep {
        SET_PALETE_POSITION,
        WAIT_FOR_PALETE,
        MOVE_ONOFREI_OUT,
        WAIT_FOR_ONOFREI,
        MOVE_ONOFREI_IN,
        WAIT_FOR_ONOFREI_RETURN,
        INCREMENT_BALL
    }

    private LaunchStep currentStep;

    public LaunchAllBalls(RobotStorage robotStorage, TelemetryManager telemetry, PaleteSubsytem paleteSubsytem,
                          OnofreiSubsystem onofreiSubsystem, LauncherSubsystem launcherSubsystem, LimelightSubsystem limelightSubsystem) {
        this.palete = paleteSubsytem;
        this.onofrei = onofreiSubsystem;
        this.launcher = launcherSubsystem;
        this.robotStorage = robotStorage;
        this.telemetry = telemetry;
        this.limelight = limelightSubsystem;

        addRequirements(palete, onofrei, launcher, limelight);
    }

    public LaunchAllBalls(RobotStorage robotStorage, TelemetryManager telemetry, PaleteSubsytem paleteSubsytem,
                          OnofreiSubsystem onofreiSubsystem, LauncherSubsystem launcherSubsystem, double launcherSpeed) {
        this.palete = paleteSubsytem;
        this.onofrei = onofreiSubsystem;
        this.launcher = launcherSubsystem;
        this.robotStorage = robotStorage;
        this.telemetry = telemetry;
        this.limelight = null;
        this.launcherSpeed = launcherSpeed;

        addRequirements(palete, onofrei, launcher);
    }

    private int sector;

    @Override
    public void initialize() {
        done = false;
        start = false;
        if (limelight != null) {
            double distance = limelight.getDistanceToDepot();
            if (distance <= 0) {
                launcherSpeed = 1350;
                telemetry.addLine("NO APRIL TAG DETECTED, FALLBACK POWER");
            } else {
                launcherSpeed = robotStorage.getLauncherSpeedForDistance(distance);
            }
        }
        launcher.spin(launcherSpeed);
        sector = 0;
        currentStep = LaunchStep.SET_PALETE_POSITION;
    }

    @Override
    public void execute() {
        launcher.spin(launcherSpeed); // trebuie apelata constant pentru pid

        if (sector > 2) {
            done = true;
            return;
        }

//        if (robotStorage.getSectorColor(sector) == 0) {
//            sector++;
//            return;
//        }

        if (launcher.atTargetSpeed()) {
            start = true;
        }

        switch (currentStep) {
            case SET_PALETE_POSITION:
                // End the command if there are no more balls to launch.
                if (sector < 0 || sector > 2) {
                    done = true;
                    break;
                }

                switch (sector) {
                    case 0:
                        palete.setPosition(PaleteSubsytem.OUT_BILA_0);
                        break;
                    case 1:
                        palete.setPosition(PaleteSubsytem.OUT_BILA_1);
                        break;
                    case 2:
                        palete.setPosition(PaleteSubsytem.OUT_BILA_2);
                        break;
                    default:
                        // Invalid sector, end the command gracefully.
                        done = true;
                        break;
                }
                if (!done && start) {
                    paleteTimer.start();
                    currentStep = LaunchStep.WAIT_FOR_PALETE;
                }
                break;

            case WAIT_FOR_PALETE:
                if (paleteTimer.done() && launcher.atTargetSpeed()) {
                    currentStep = LaunchStep.MOVE_ONOFREI_OUT;
                }
                break;

            case MOVE_ONOFREI_OUT:
                onofrei.setPosition(OnofreiSubsystem.OUT);
                telemetry.addLine("ONOFREI OUT");
                onofreiTimer.start();
                currentStep = LaunchStep.WAIT_FOR_ONOFREI;
                break;

            case WAIT_FOR_ONOFREI:
                if (onofreiTimer.done()) {
                    currentStep = LaunchStep.MOVE_ONOFREI_IN;
                }
                break;

            case MOVE_ONOFREI_IN:
                onofrei.setPosition(OnofreiSubsystem.IN);
                onofreiTimer.start(); // Start timer to wait for Onofrei to return
                currentStep = LaunchStep.WAIT_FOR_ONOFREI_RETURN;
                break;

            case WAIT_FOR_ONOFREI_RETURN: // New state
                if (onofreiTimer.done()) {
                    currentStep = LaunchStep.INCREMENT_BALL;
                }
                break;

            case INCREMENT_BALL:
                robotStorage.setSector(sector, 0);
                sector++;
                // Move to the next ball
                currentStep = LaunchStep.SET_PALETE_POSITION;
                break;
        }


        telemetry.addData("sector", sector);
        telemetry.addData("step", currentStep.name());
        telemetry.addData("done", done);
        telemetry.addData("start", start);
        telemetry.addData("flywheel speed", launcher.getVelocity());
        telemetry.addData("flywheel target speed", launcherSpeed);
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
