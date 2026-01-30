package org.firstinspires.ftc.teamcode.commands;

import com.bylazar.telemetry.TelemetryManager;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.Timing.Timer;

import org.firstinspires.ftc.teamcode.subsystems.IntakeKickerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SenzorGauraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SenzorTavanSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PaleteSubsytem;
import org.firstinspires.ftc.teamcode.subsystems.RobotStorage;

import java.util.concurrent.TimeUnit;

public class IntakeBall extends CommandBase {
    private final Timer timerPalete = new Timer(400, TimeUnit.MILLISECONDS);
    private final Timer timerKicker = new Timer(100, TimeUnit.MILLISECONDS);
    private final IntakeSubsystem intake;
    private final IntakeKickerSubsystem kicker;
    private final PaleteSubsytem palete;
    private final SenzorTavanSubsystem senzorTavan;
    private final SenzorGauraSubsystem senzorGaura;
    private final RobotStorage robotStorage;
    private final TelemetryManager telemetry;

    private int sector = -2;

    private enum IntakeStep {
        POSITION_PALETE,
        WAIT_FOR_BALL,
        STORE_BALL,
        DONE
    }

    private IntakeStep currentStep;

    public IntakeBall(RobotStorage robotStorage, TelemetryManager telemetry, IntakeSubsystem intakeSubsystem, PaleteSubsytem paleteSubsytem,
                      SenzorTavanSubsystem senzorTavanSubsystem, SenzorGauraSubsystem senzorGauraSubsystem, IntakeKickerSubsystem intakeKickerSubsystem) {
        this.intake = intakeSubsystem;
        this.palete = paleteSubsytem;
        this.senzorTavan = senzorTavanSubsystem;
        this.senzorGaura = senzorGauraSubsystem;
        this.kicker = intakeKickerSubsystem;
        this.robotStorage = robotStorage;
        this.telemetry = telemetry;

        addRequirements(intake, palete, senzorTavan, senzorGaura, kicker);
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
                intake.suck();

                if (senzorTavan.getDistanceMM() < 25 || senzorGaura.getDistanceMM() < 25) {
                    /// metoda veche
//                    timerKicker.start();
//                    currentStep = IntakeStep.STORE_BALL;
                    kicker.setPosition(IntakeKickerSubsystem.OUT);
                    currentStep = IntakeStep.STORE_BALL;
                }

                break;

            case STORE_BALL:
                /// metoda veche
//                if (timerKicker.done()) {
//                    kicker.setPosition(IntakeKickerSubsystem.OUT);
//                }
//                if (senzorTavan.getDistanceMM() > 60) {
//                    robotStorage.setSector(sector, senzorGaura.getColor());
//                    timerPalete.start();
//                    sector = robotStorage.getNextFreeSector();
//                    currentStep = IntakeStep.POSITION_PALETE;
//                }
                robotStorage.setSector(sector, senzorGaura.getColor());
                timerPalete.start();
                sector = robotStorage.getNextFreeSector();
                currentStep = IntakeStep.POSITION_PALETE;

                break;

            case DONE:
                // Command will finish.
                break;
        }

        telemetry.addData("sector", sector);
        telemetry.addData("culoare", senzorGaura.getHSVColor()[0]);
        telemetry.addData("distanta tavan", senzorTavan.getDistanceMM());
        telemetry.addData("distanta gaura", senzorGaura.getDistanceMM());
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
