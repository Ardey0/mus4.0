package org.firstinspires.ftc.teamcode.commands;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OnofreiSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PaleteSubsytem;
import org.firstinspires.ftc.teamcode.subsystems.RampaSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotStorage;

import java.util.concurrent.TimeUnit;
import java.util.function.DoubleSupplier;

public class LaunchBallByColor extends CommandBase {
    private final int alliance; // 0 - albastru, 1 - rosu
    private final Follower follower;
    private final OnofreiSubsystem onofrei;
    private final PaleteSubsytem palete;
    private final LauncherSubsystem launcher;
    private final RampaSubsystem rampa;
    private final RobotStorage robotStorage;
    private final TelemetryManager telemetry;
    private final Timing.Timer onofreiTimer = new Timing.Timer(200, TimeUnit.MILLISECONDS);
    private final Timing.Timer paleteTimer = new Timing.Timer(400, TimeUnit.MILLISECONDS);
    private final int color;
    private boolean done = false, start = false;

    private final DoubleSupplier launcherSpeedSupplier;
    private final DoubleSupplier rampAngleSupplier;

    // State machine for the launching sequence
    private enum LaunchStep {
        SET_PALETE_POSITION,
        WAIT_FOR_PALETE,
        MOVE_ONOFREI_OUT,
        WAIT_FOR_ONOFREI,
        MOVE_ONOFREI_IN,
        WAIT_FOR_ONOFREI_RETURN, // Added state for delay
        INCREMENT_BALL
    }

    private LaunchStep currentStep;

    public LaunchBallByColor(RobotStorage robotStorage, TelemetryManager telemetry, Follower follower, PaleteSubsytem paleteSubsytem,
                          OnofreiSubsystem onofreiSubsystem, LauncherSubsystem launcherSubsystem, RampaSubsystem rampaSubsystem,
                          int alliance, int color) {
        this.palete = paleteSubsytem;
        this.onofrei = onofreiSubsystem;
        this.launcher = launcherSubsystem;
        this.rampa = rampaSubsystem;
        this.robotStorage = robotStorage;
        this.telemetry = telemetry;
        this.alliance = alliance;
        this.follower = follower;
        this.color = color;
        this.launcherSpeedSupplier = () -> 0; // Should not be used because follower is not null
        this.rampAngleSupplier = () -> 0; // Should not be used because follower is not null

        addRequirements(palete, onofrei, launcher, rampa);
    }

    public LaunchBallByColor(RobotStorage robotStorage, TelemetryManager telemetry, PaleteSubsytem paleteSubsytem,
                          OnofreiSubsystem onofreiSubsystem, LauncherSubsystem launcherSubsystem, RampaSubsystem rampaSubsystem,
                          double launcherSpeed, double rampAngle, int alliance, int color) {
        this.palete = paleteSubsytem;
        this.onofrei = onofreiSubsystem;
        this.launcher = launcherSubsystem;
        this.rampa = rampaSubsystem;
        this.robotStorage = robotStorage;
        this.telemetry = telemetry;
        this.launcherSpeedSupplier = () -> launcherSpeed;
        this.rampAngleSupplier = () -> rampAngle;
        this.alliance = alliance;
        this.follower = null;
        this.color = color;

        addRequirements(palete, onofrei, launcher, rampa);
    }

    public LaunchBallByColor(RobotStorage robotStorage, TelemetryManager telemetry, PaleteSubsytem paleteSubsytem,
                             OnofreiSubsystem onofreiSubsystem, LauncherSubsystem launcherSubsystem, RampaSubsystem rampaSubsystem,
                             DoubleSupplier launcherSpeed, DoubleSupplier rampAngleSupplier, int alliance, int color) {
        this.palete = paleteSubsytem;
        this.onofrei = onofreiSubsystem;
        this.launcher = launcherSubsystem;
        this.rampa = rampaSubsystem;
        this.robotStorage = robotStorage;
        this.telemetry = telemetry;
        this.launcherSpeedSupplier = launcherSpeed;
        this.rampAngleSupplier = rampAngleSupplier;
        this.alliance = alliance;
        this.follower = null;
        this.color = color;

        addRequirements(palete, onofrei, launcher, rampa);
    }

    private int sector;

    @Override
    public void initialize() {
        done = false;
        start = false;
        launcher.spin(getLauncherSpeed());
        currentStep = LaunchStep.SET_PALETE_POSITION;
        sector = robotStorage.getNextSectorWithColor(color);
    }

    @Override
    public void execute() {
        launcher.spin(getLauncherSpeed());
        rampa.setPosition(getRampAngle());

        if (launcher.atTargetSpeed()) {
            start = true;
        }

        switch (currentStep) {
            case SET_PALETE_POSITION:
                // End the command if there are no more balls with motifs to launch.
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
                done = true;
                break;
        }

        telemetry.addData("sector", sector);
        telemetry.addData("step", currentStep.name());
        telemetry.addData("done", done);
        telemetry.addData("flywheel speed", launcher.getVelocity());
        telemetry.addData("flywheel target speed", getLauncherSpeed());
        telemetry.addData("ramp angle", getRampAngle());
    }

    @Override
    public void end(boolean interrupted) {
        launcher.stop();
        onofrei.setPosition(OnofreiSubsystem.IN);
        rampa.setPosition(0);
    }

    @Override
    public boolean isFinished() {
        return done && onofreiTimer.done();
    }

    private double getLauncherSpeed() {
        if (follower == null) {
            return launcherSpeedSupplier.getAsDouble();
        }

        return robotStorage.getLauncherSpeedForDist();
    }

    private double getRampAngle() {
        if (follower == null) {
            return rampAngleSupplier.getAsDouble();
        }

        return robotStorage.getRampAngleForDist();
    }
}
