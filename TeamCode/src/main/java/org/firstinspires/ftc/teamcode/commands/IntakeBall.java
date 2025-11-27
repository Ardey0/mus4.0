package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.Timing.Timer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PaleteSubsytem;
import org.firstinspires.ftc.teamcode.subsystems.RoataSubsystem;

import java.util.concurrent.TimeUnit;

public class IntakeBall extends CommandBase {
    private Timer timerPalete = new Timer(500, TimeUnit.MILLISECONDS);
    private final IntakeSubsystem intake;
    private final PaleteSubsytem palete;
    private final ColorSensorSubsystem sensor;
    private final RoataSubsystem roata;
    private final Telemetry telemetry;
    private int sector = -2;

    public IntakeBall(IntakeSubsystem intakeSubsystem, PaleteSubsytem paleteSubsytem,
                      ColorSensorSubsystem colorSensorSubsystem, RoataSubsystem roataSubsystem, Telemetry telemetry) {
        this.intake = intakeSubsystem;
        this.palete = paleteSubsytem;
        this.sensor = colorSensorSubsystem;
        this.roata = roataSubsystem;
        this.telemetry = telemetry;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.suck();
        timerPalete.start();
    }

    @Override
    public void execute() {
        if (timerPalete.done()) {
            sector = roata.getNextFreeSector();
            switch (sector) {
                case -1:
                    palete.setPosition(PaleteSubsytem.LOCK);
                    break;
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
                    telemetry.update();
            }
            if (sector >= 0 && sector <= 2) {
                if (sensor.getDistanceMM() < 20) {
                    roata.setSector(sector, sensor.getColorName());
                    timerPalete.start();
                }
            }
        }
        telemetry.addData("klsdflkgarkljghhkaerg;k: ", sector);
        telemetry.addData("culoare sector 1: ", roata.getSectorColor(0));
        telemetry.addData("culoare sector 2: ", roata.getSectorColor(1));
        telemetry.addData("culoare sector 3: ", roata.getSectorColor(2));
        telemetry.addData("timer: ", timerPalete.remainingTime());
        telemetry.addData("timer done? ", timerPalete.done());
        telemetry.addData("timer on? ", timerPalete.isTimerOn());
        telemetry.update();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        // paltete pe o pozitie unde tine bilele sa nu iasa pe nicaieri
    }

    @Override
    public boolean isFinished() {
        return sector == -1;
    }

}
