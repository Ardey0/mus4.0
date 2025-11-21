package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.SensorRevColorV3;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ColorSensorSubsystem extends SubsystemBase {
    private final SensorRevColorV3 colorSensor;

    public ColorSensorSubsystem(HardwareMap hwMap){
        this.colorSensor = new SensorRevColorV3(hwMap, "senzor");
    }

    public int[] getRGBColor() {
        return colorSensor.getARGB();
    }

    public float[] getHSVColor() {
        float[] HSVColors = new float[3];
        colorSensor.RGBtoHSV(colorSensor.red(), colorSensor.green(), colorSensor.blue(), HSVColors);

        return HSVColors;
    }

    public String getColorName() {
        float[] HSVColor = getHSVColor();
        if (HSVColor[0] > 80 && HSVColor[0] < 140) {
            return "GREEN";
        }
        if (HSVColor[0] > 220 && HSVColor[0] < 330){
            return "PURPLE";
        }
        return null;
    }

    public double getDistanceMM() {
        return colorSensor.distance(DistanceUnit.MM);
    }
}
