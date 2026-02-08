package org.firstinspires.ftc.teamcode.commands;

import com.bylazar.telemetry.TelemetryManager;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.subsystems.IntakeKickerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PaleteSubsytem;
import org.firstinspires.ftc.teamcode.subsystems.RobotStorage;
import org.firstinspires.ftc.teamcode.subsystems.SenzorGauraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SenzorRoataSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SenzorTavanSubsystem;

import java.util.concurrent.TimeUnit;

public class IntakeBall extends CommandBase {
    private final Timing.Timer timerPalete = new Timing.Timer(500, TimeUnit.MILLISECONDS);
    private final Timing.Timer timerCuloare = new Timing.Timer(100, TimeUnit.MILLISECONDS);
    private final Timing.Timer timerKicker = new Timing.Timer(50, TimeUnit.MILLISECONDS);
    private final Timing.Timer timerFail = new Timing.Timer(500, TimeUnit.MILLISECONDS);
    private final IntakeSubsystem intake;
    private final IntakeKickerSubsystem kicker;
    private final PaleteSubsytem palete;
    private final SenzorTavanSubsystem senzorTavan;
    private final SenzorGauraSubsystem senzorGaura;
    private final SenzorRoataSubsystem senzorRoata;
    private final RobotStorage robotStorage;
    private final TelemetryManager telemetry;

    private int sector = -2, colorReadings = 0;
    private float green = 0, purple = 0;

    private enum IntakeStep {
        POSITION_PALETE,
        WAIT_FOR_BALL,
        STORE_BALL,
        DONE
    }

    private IntakeStep currentStep;

    public IntakeBall(RobotStorage robotStorage, TelemetryManager telemetry, IntakeSubsystem intakeSubsystem, PaleteSubsytem paleteSubsytem,
                      SenzorTavanSubsystem senzorTavanSubsystem, SenzorRoataSubsystem senzorRoataSubsystem, SenzorGauraSubsystem senzorGauraSubsystem, IntakeKickerSubsystem intakeKickerSubsystem) {
        this.intake = intakeSubsystem;
        this.palete = paleteSubsytem;
        this.senzorTavan = senzorTavanSubsystem;
        this.senzorRoata = senzorRoataSubsystem;
        this.senzorGaura = senzorGauraSubsystem;
        this.kicker = intakeKickerSubsystem;
        this.robotStorage = robotStorage;
        this.telemetry = telemetry;

        addRequirements(intake, palete, senzorTavan, senzorRoata, senzorGaura, kicker);
    }

    @Override
    public void initialize() {
        kicker.setPosition(IntakeKickerSubsystem.IN);
        timerPalete.start();
        sector = robotStorage.getNextFreeSector();
        currentStep = IntakeStep.POSITION_PALETE;
    }

    @Override
    public void execute() {
        switch (currentStep) {
            case POSITION_PALETE:
                if (sector == -1) {
                    palete.setPosition(PaleteSubsytem.LOCK);
                    intake.stop();
                    currentStep = IntakeStep.DONE;
                    break;
                }

                switch (sector) {
                    case 0:
                        palete.setPosition(PaleteSubsytem.IN_BILA_0);
                        break;
                    case 1:
                        palete.setPosition(PaleteSubsytem.IN_BILA_1);
                        break;
                    case 2:
                        palete.setPosition(PaleteSubsytem.IN_BILA_2);
                        break;
                    default:
                        telemetry.addData("problema roata; sector", sector);
                        break;
                }
                if (timerPalete.done()) {
                    currentStep = IntakeStep.WAIT_FOR_BALL;
                }
                break;

            case WAIT_FOR_BALL:
                kicker.setPosition(IntakeKickerSubsystem.IN);
                intake.suck(1);

                if (senzorTavan.getDistanceMM() < 30 || senzorRoata.getDistanceMM() < 55) {
                    timerKicker.start();
                    timerFail.start();
                    currentStep = IntakeStep.STORE_BALL;
                }
                break;

            case STORE_BALL:
                intake.suck(0.75);
                if (timerKicker.done()) {
                    kicker.setPosition(IntakeKickerSubsystem.OUT);
                }
                if (senzorRoata.getDistanceMM() < 65) {
                    robotStorage.setSector(sector, 2);
                    sector = robotStorage.getNextFreeSector();
                    timerPalete.start();
                    currentStep = IntakeStep.POSITION_PALETE;
                    break;
                }
                if (timerFail.done() && senzorTavan.getDistanceMM() < 35) {
                    currentStep = IntakeStep.WAIT_FOR_BALL;
                    break;
                }
                break;

            case DONE:
                // Command will finish.
                break;
        }

        telemetry.addData("sector", sector);
//        telemetry.addLine("");
//        telemetry.addData("distanta tavan", senzorTavan.getDistanceMM());
//        telemetry.addData("distanta roata", senzorRoata.getDistanceMM());
//        telemetry.addLine("");
//        telemetry.addData("red", senzorRoata.getRed());
//        telemetry.addData("blue", senzorRoata.getBlue());
//        telemetry.addData("green", senzorRoata.getGreen());
//        telemetry.addLine("");
        telemetry.addData("culoare sector 0", robotStorage.getSectorColor(0));
        telemetry.addData("culoare sector 1", robotStorage.getSectorColor(1));
        telemetry.addData("culoare sector 2", robotStorage.getSectorColor(2));
        telemetry.addData("step", currentStep.name());
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        kicker.setPosition(IntakeKickerSubsystem.IN);
        palete.setPosition(PaleteSubsytem.LOCK);
    }

    @Override
    public boolean isFinished() {
        return currentStep == IntakeStep.DONE && timerPalete.done();
    }
}
