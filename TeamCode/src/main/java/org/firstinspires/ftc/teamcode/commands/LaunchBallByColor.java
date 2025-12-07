package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OnofreiSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PaleteSubsytem;
import org.firstinspires.ftc.teamcode.subsystems.RobotStorage;

import java.util.concurrent.TimeUnit;
import java.util.function.Supplier;

public class LaunchBallByColor extends CommandBase {
    private final OnofreiSubsystem onofrei;
    private final PaleteSubsytem palete;
    private final LauncherSubsystem launcher;
    private final RobotStorage robotStorage;
    private final Telemetry telemetry;
    private final Timing.Timer onofreiTimer = new Timing.Timer(500, TimeUnit.MILLISECONDS);
    private final Timing.Timer paleteTimer = new Timing.Timer(600, TimeUnit.MILLISECONDS);
    private final Timing.Timer flywheelTimer = new Timing.Timer(2000, TimeUnit.MILLISECONDS);
    private final Supplier<Boolean> launchFromFar;
    private double targetSpeed;
    private final int color;
    private boolean done = false;

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

    public LaunchBallByColor(RobotStorage robotStorage, Telemetry telemetry, PaleteSubsytem paleteSubsytem,
                             OnofreiSubsystem onofreiSubsystem, LauncherSubsystem launcherSubsystem,
                             Supplier<Boolean> launchFromFar, int color) {
        this.palete = paleteSubsytem;
        this.onofrei = onofreiSubsystem;
        this.launcher = launcherSubsystem;
        this.robotStorage = robotStorage;
        this.telemetry = telemetry;
        this.launchFromFar = launchFromFar;
        this.color = color;

        addRequirements(palete, onofrei, launcher);
    }

    private int sector;

    @Override
    public void initialize() {
        done = false;
        targetSpeed = launchFromFar.get() ? LauncherSubsystem.FAR_TARGET_SPEED : LauncherSubsystem.NEAR_TARGET_SPEED;
        launcher.spin(targetSpeed);
        flywheelTimer.start();
        currentStep = LaunchStep.SET_PALETE_POSITION;
        sector = robotStorage.getNextSectorWithColor(color);
    }

    @Override
    public void execute() {
        launcher.spin(targetSpeed); // trebuie apelata constant pentru pid

        if (launcher.atTargetSpeed()) {
            switch (currentStep) {
                case SET_PALETE_POSITION:
                    // End the command if there are no more balls with motifs to launch.
                    if (sector < 0 || sector > 2) {
                        done = true;
                        break;
                    }

                    switch (sector) {
                        case 0:
                            palete.setPosition(PaleteSubsytem.OUT_BILA_1);
                            break;
                        case 1:
                            palete.setPosition(PaleteSubsytem.OUT_BILA_2);
                            break;
                        case 2:
                            palete.setPosition(PaleteSubsytem.OUT_BILA_3);
                            break;
                        default:
                            // Invalid sector, end the command gracefully.
                            done = true;
                            break;
                    }

                    if (!done) {
                        paleteTimer.start();
                        currentStep = LaunchStep.WAIT_FOR_PALETE;
                    }
                    break;

                case WAIT_FOR_PALETE:
                    if (paleteTimer.done()) {
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
        }

        telemetry.addData("sector:", sector);
        telemetry.addData("step:", currentStep.name());
        telemetry.addData("done:", done);
        telemetry.addData("flywheel speed", launcher.getVelocity());
        telemetry.addData("flywheel target speed", targetSpeed);
        telemetry.update();
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
