package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.Timing.Timer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PaleteSubsytem;
import org.firstinspires.ftc.teamcode.subsystems.RobotStorage;

import java.util.concurrent.TimeUnit;

public class IntakeBall extends CommandBase {
    private final Timer timerPalete = new Timer(100, TimeUnit.MILLISECONDS);
    private final IntakeSubsystem intake;
    private final PaleteSubsytem palete;
    private final ColorSensorSubsystem sensor;
    private final RobotStorage robotStorage;
    private final Telemetry telemetry;

    private int sector = -2;

    private enum IntakeStep {
        POSITION_PALETE,
        WAIT_FOR_BALL,
        STORE_BALL,
        WAIT_AND_CYCLE,
        DONE
    }
    private IntakeStep currentStep;

    public IntakeBall(RobotStorage robotStorage, Telemetry telemetry, IntakeSubsystem intakeSubsystem, PaleteSubsytem paleteSubsytem,
                      ColorSensorSubsystem colorSensorSubsystem) {
        this.intake = intakeSubsystem;
        this.palete = paleteSubsytem;
        this.sensor = colorSensorSubsystem;
        this.robotStorage = robotStorage;
        this.telemetry = telemetry;

        addRequirements(intake, palete, sensor);
    }

    @Override
    public void initialize() {
        intake.suck();
        timerPalete.start();
        currentStep = IntakeStep.POSITION_PALETE;
    }

    @Override
    public void execute() {
        switch (currentStep) {
            case POSITION_PALETE:
                sector = robotStorage.getNextFreeSector();

                if (sector == -1) {
                    palete.setPosition(PaleteSubsytem.LOCK);
                    intake.stop();
                    currentStep = IntakeStep.DONE;
                    break;
                }

                intake.suck(); // Keep intake running
                switch (sector) {
                    case 0:
                        palete.setPosition(PaleteSubsytem.IN_BILA_1);
                        break;
                    case 1:
                        palete.setPosition(PaleteSubsytem.IN_BILA_2);
                        break;
                    case 2:
                        palete.setPosition(PaleteSubsytem.IN_BILA_3);
                        break;
                    default:
                        telemetry.addData("problema roata; sector:", sector);
                        break;
                }
                currentStep = IntakeStep.WAIT_FOR_BALL;
                break;

            case WAIT_FOR_BALL:
                if (sensor.getDistanceMM() < 30) {
                    currentStep = IntakeStep.STORE_BALL;
                }
                break;

            case STORE_BALL:
                robotStorage.setSector(sector, sensor.getColor());
                intake.stop();
                if (sensor.getDistanceMM() > 55) {
                    timerPalete.start();
                    currentStep = IntakeStep.WAIT_AND_CYCLE;
                }
                break;

            case WAIT_AND_CYCLE:
                if (timerPalete.done() && sensor.getDistanceMM() > 55) {
                    currentStep = IntakeStep.POSITION_PALETE;
                }
                break;

            case DONE:
                // Command will finish.
                break;
        }

        telemetry.addData("sector:", sector);
        telemetry.addData("culoare:", sensor.getHSVColor()[0]);
        telemetry.addData("distanta:", sensor.getDistanceMM());
        telemetry.addData("culoare sector 0:", robotStorage.getSectorColor(0));
        telemetry.addData("culoare sector 1:", robotStorage.getSectorColor(1));
        telemetry.addData("culoare sector 2:", robotStorage.getSectorColor(2));
        telemetry.addData("timer remaining:", timerPalete.remainingTime());
        telemetry.addData("step:", currentStep.name());
        telemetry.update();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        palete.setPosition(PaleteSubsytem.LOCK);
    }

    @Override
    public boolean isFinished() {
        return currentStep == IntakeStep.DONE && timerPalete.done();
    }

}
