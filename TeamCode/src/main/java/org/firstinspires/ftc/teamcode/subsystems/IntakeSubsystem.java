package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.SensorRevColorV3;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IntakeSubsystem extends SubsystemBase {
    private final Motor intake;
    private final SensorRevColorV3 colorSensor;
    public IntakeSubsystem(HardwareMap hwMap) {
        this.colorSensor = new SensorRevColorV3(hwMap, "senzor");
        this.intake = new Motor(hwMap, "intake");
        this.intake.setInverted(true);
        this.intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.intake.setRunMode(Motor.RunMode.RawPower);
    }

    public void suck() {
        intake.set(1);
    }

    public void stop() {
        intake.set(0);
    }

    public int[] getRGBColor() {
        return colorSensor.getARGB();
    }

    public float[] getHSVColor() {
        float[] HSVColors = new float[3];
        colorSensor.RGBtoHSV(colorSensor.red(), colorSensor.green(), colorSensor.blue(), HSVColors);

        return HSVColors;
    }

    public double getDistanceMM() {
        return colorSensor.distance(DistanceUnit.MM);
    }
}

