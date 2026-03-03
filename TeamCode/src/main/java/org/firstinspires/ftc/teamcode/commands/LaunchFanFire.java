package org.firstinspires.ftc.teamcode.commands;

import static com.seattlesolvers.solverslib.util.MathUtils.clamp;

import com.bylazar.configurables.annotations.Configurable;
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

@Configurable
public class LaunchFanFire extends CommandBase {
    public static double coefVit = 50, coefUng = 0;
    private final int alliance; // 0 - albastru, 1 - rosu
    private final Follower follower;
    private final OnofreiSubsystem onofrei;
    private final PaleteSubsytem palete;
    private final LauncherSubsystem launcher;
    private final RampaSubsystem rampa;
    private final RobotStorage robotStorage;
    private final TelemetryManager telemetry;

    private final Timer paleteTimer = new Timer(200, TimeUnit.MILLISECONDS);
    private final Timer launchTimer = new Timer(800, TimeUnit.MILLISECONDS);
    private final Timer onofreiOutTimer = new Timer(70, TimeUnit.MILLISECONDS);

    private boolean done = false;

    private final DoubleSupplier launcherSpeedSupplier;
    private final DoubleSupplier rampAngleSupplier;

    private enum LaunchStep {
        SET_PALETE_POSITION,
        WAIT_FOR_PALETE,
        MOVE_ONOFREI_OUT,
        WAIT_FOR_ONOFREI,
        LAUNCH
    }

    private LaunchStep currentStep;

    public LaunchFanFire(RobotStorage robotStorage, TelemetryManager telemetry, Follower follower, PaleteSubsytem paleteSubsytem,
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

    public LaunchFanFire(RobotStorage robotStorage, TelemetryManager telemetry, PaleteSubsytem paleteSubsytem,
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

    public LaunchFanFire(RobotStorage robotStorage, TelemetryManager telemetry, PaleteSubsytem paleteSubsytem,
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

    public LaunchFanFire(RobotStorage robotStorage, TelemetryManager telemetry, PaleteSubsytem paleteSubsytem,
                         OnofreiSubsystem onofreiSubsystem, LauncherSubsystem launcherSubsystem, RampaSubsystem rampaSubsystem,
                         DoubleSupplier launcherSpeed, int alliance) {
        this.palete = paleteSubsytem;
        this.onofrei = onofreiSubsystem;
        this.launcher = launcherSubsystem;
        this.rampa = rampaSubsystem;
        this.robotStorage = robotStorage;
        this.telemetry = telemetry;
        this.launcherSpeedSupplier = launcherSpeed;
        this.rampAngleSupplier = () -> 0;
        this.alliance = alliance;
        this.follower = null;

        addRequirements(palete, onofrei, launcher, rampa);
    }

    @Override
    public void initialize() {
        done = false;
        launcher.spin(getLauncherSpeed());
        currentStep = LaunchStep.SET_PALETE_POSITION;
    }

    @Override
    public void execute() {
        updateDistanceToGoal();
        launcher.spin(getLauncherSpeed());
        rampa.setPosition(getRampAngle());

        switch (currentStep) {
            case SET_PALETE_POSITION:
                palete.setPosition(PaleteSubsytem.LOCK);
                paleteTimer.start();
                currentStep = LaunchStep.WAIT_FOR_PALETE;
                break;

            case WAIT_FOR_PALETE:
                if (paleteTimer.done()) {
                    currentStep = LaunchStep.MOVE_ONOFREI_OUT;
                }
                break;

            case MOVE_ONOFREI_OUT:
                onofrei.setPosition(OnofreiSubsystem.FAN_FIRE);
                onofreiOutTimer.start();
                currentStep = LaunchStep.WAIT_FOR_ONOFREI;
                break;

            case WAIT_FOR_ONOFREI:
                if (onofreiOutTimer.done() && launcher.atTargetSpeed()) {
                    launchTimer.start();
                    currentStep = LaunchStep.LAUNCH;
                }
                break;

            case LAUNCH:
                palete.setPosition(PaleteSubsytem.FAN_FIRE);
                done = true;
                break;
        }

        telemetry.addData("step", currentStep.name());
//        telemetry.addData("done", done);
//        telemetry.addData("start", start);
        telemetry.addData("flywheel speed", launcher.getVelocity());
        telemetry.addData("flywheel target speed", getLauncherSpeed());
        telemetry.addData("ramp angle", getRampAngle());
    }

    @Override
    public void end(boolean interrupted) {
        for (int i = 0; i < 3; i++)
            robotStorage.setSector(i, 0);
        onofrei.setPosition(OnofreiSubsystem.IN);
        palete.setPosition(PaleteSubsytem.LOCK);
        launcher.stop();
    }

    @Override
    public boolean isFinished() {
        return done && launchTimer.done();
    }

    private void updateDistanceToGoal() {
        if (follower == null) {
            return;
        }
        robotStorage.updateDistanceToGoal(follower.getPose().getX(), follower.getPose().getY(), alliance);
    }

    private double getLauncherSpeed() {
        if (follower == null) {
            return launcherSpeedSupplier.getAsDouble();
        }

        return clamp(robotStorage.getLauncherSpeedForDist() + coefVit, 0, 1900);
    }

    private double getRampAngle() {
//        if (follower == null) {
//            return rampAngleSupplier.getAsDouble();
//        }

        double launcherSpeed = launcher.getVelocity();
        double d2 = launcherSpeed * launcherSpeed;
        double d3 = d2 * launcherSpeed;
        double d4 = d3 * launcherSpeed;
        return clamp(1.40164e-11 * d4 - 8.01586e-8 * d3 +
                        0.000168479 * d2 -
                        0.152832 * launcherSpeed + 50.59207
                        + coefUng,
                0.2, 0.92);
    }
}
