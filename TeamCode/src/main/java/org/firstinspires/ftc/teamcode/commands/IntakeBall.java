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
        GET_BALL_COLOR,
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
}
