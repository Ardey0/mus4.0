package org.firstinspires.ftc.teamcode.commands;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.Timing.Timer;

import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OnofreiSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PaleteSubsytem;
import org.firstinspires.ftc.teamcode.subsystems.RampaSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotStorage;

import java.util.concurrent.TimeUnit;
import java.util.function.DoubleSupplier;

public class LaunchMotifBalls extends CommandBase {
    private final int alliance; // 0 - albastru, 1 - rosu
    private final Follower follower;
    private final OnofreiSubsystem onofrei;
    private final PaleteSubsytem palete;
    private final LauncherSubsystem launcher;
    private final RampaSubsystem rampa;
    private final RobotStorage robotStorage;
    private final TelemetryManager telemetry;
    private final Timer onofreiTimer = new Timer(200, TimeUnit.MILLISECONDS);
    private final Timer paleteTimer = new Timer(400, TimeUnit.MILLISECONDS);
    private int ball = 0;
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

    public LaunchMotifBalls(RobotStorage robotStorage, TelemetryManager telemetry, Follower follower, PaleteSubsytem paleteSubsytem,
                          OnofreiSubsystem onofreiSubsystem, LauncherSubsystem launcherSubsystem, RampaSubsystem rampaSubsystem,
                          int alliance) {
        this.palete = paleteSubsytem;
        this.onofrei = onofreiSubsystem;
        this.launcher = launcherSubsystem;
        this.rampa = rampaSubsystem;
        this.robotStorage = robotStorage;
        this.telemetry = telemetry;
        this.alliance = alliance;
        this.follower = follower;
        this.launcherSpeedSupplier = () -> 0; // Should not be used because follower is not null
        this.rampAngleSupplier = () -> 0; // Should not be used because follower is not null

        addRequirements(palete, onofrei, launcher, rampa);
    }

    public LaunchMotifBalls(RobotStorage robotStorage, TelemetryManager telemetry, PaleteSubsytem paleteSubsytem,
                          OnofreiSubsystem onofreiSubsystem, LauncherSubsystem launcherSubsystem, RampaSubsystem rampaSubsystem,
                          double launcherSpeed, double rampAngle, int alliance) {
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

        addRequirements(palete, onofrei, launcher, rampa);
    }

    public LaunchMotifBalls(RobotStorage robotStorage, TelemetryManager telemetry, PaleteSubsytem paleteSubsytem,
                          OnofreiSubsystem onofreiSubsystem, LauncherSubsystem launcherSubsystem, RampaSubsystem rampaSubsystem,
                          DoubleSupplier launcherSpeed, DoubleSupplier rampAngleSupplier, int alliance) {
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

        addRequirements(palete, onofrei, launcher, rampa);
    }


    @Override
    public void initialize() {
        done = false;
        start = false;
        launcher.spin(getLauncherSpeed());
        ball = 0;
        currentStep = LaunchStep.SET_PALETE_POSITION;
    }

    @Override
    public void execute() {
        launcher.spin(getLauncherSpeed());
        rampa.setPosition(getRampAngle());

        int sector = robotStorage.getNextSectorWithMotifBall(ball);

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
                telemetry.addLine("ONOFREI OUT");
                onofreiTimer.start();
                currentStep = LaunchStep.WAIT_FOR_ONOFREI;
                break;

            case WAIT_FOR_ONOFREI:
                if (onofreiTimer.done() && launcher.atTargetSpeed()) {
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
                ball++;
                // Move to the next ball
                currentStep = LaunchStep.SET_PALETE_POSITION;
                break;
        }

        telemetry.addData("sector", sector);
        telemetry.addData("ball", ball);
        telemetry.addData("step", currentStep.name());
        telemetry.addData("start", start);
        telemetry.addData("done", done);
        telemetry.addData("flywheel speed", launcher.getVelocity());
        telemetry.addData("flywheel target speed", getLauncherSpeed());
        telemetry.addData("ramp angle", getRampAngle());
    }

    @Override
    public void end(boolean interrupted) {
        launcher.stop();
        onofrei.setPosition(OnofreiSubsystem.IN);
    }

    @Override
    public boolean isFinished() {
        return done && onofreiTimer.done();
    }


    private double getLauncherSpeed() {
        if (follower == null) {
            return launcherSpeedSupplier.getAsDouble();
        }

        if (alliance == 0) {
            return robotStorage.getLauncherSpeedForCoordsBlue(follower.getPose().getX(), follower.getPose().getY());
        } else {
            return robotStorage.getLauncherSpeedForCoordsRed(follower.getPose().getX(), follower.getPose().getY());
        }
    }

    private double getRampAngle() {
        if (follower == null) {
            return rampAngleSupplier.getAsDouble();
        }

        if (alliance == 0) {
            return robotStorage.getRampAngleForCoordsBlue(follower.getPose().getX(), follower.getPose().getY());
        } else {
            return robotStorage.getRampAngleForCoordsRed(follower.getPose().getX(), follower.getPose().getY());
        }
    }
}
