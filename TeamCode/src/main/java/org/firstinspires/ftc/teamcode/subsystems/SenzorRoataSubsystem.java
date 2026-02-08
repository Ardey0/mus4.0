package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class SenzorRoataSubsystem extends SubsystemBase {
    private final NormalizedColorSensor colorSensor;
    private final double purpleCompensation = 0.0006;

    public SenzorRoataSubsystem(HardwareMap hwMap){
        this.colorSensor = hwMap.get(NormalizedColorSensor.class, "senzor_roata");
    }

    public float getRed() {
        return colorSensor.getNormalizedColors().red;
    }

    public float getBlue() {
        return colorSensor.getNormalizedColors().blue;
    }

    public float getGreen() {
        return colorSensor.getNormalizedColors().green;
    }

    public int getColor() { // GREEN = 1    PURPLE = 2
        return colorSensor.getNormalizedColors().blue < colorSensor.getNormalizedColors().green ? 1 : 2;
//        return colorSensor.getNormalizedColors().blue + colorSensor.getNormalizedColors().red + purpleCompensation - colorSensor.getNormalizedColors().green < 0.001 ? 1 : 2;
    }

    public double getDistanceMM() {
        return ((DistanceSensor) colorSensor).getDistance(DistanceUnit.MM);
    }
}
