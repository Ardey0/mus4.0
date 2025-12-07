package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.SensorRevColorV3;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ColorSensorSubsystem extends SubsystemBase {
    private final SensorRevColorV3 colorSensor;

    public ColorSensorSubsystem(HardwareMap hwMap){
        this.colorSensor = new SensorRevColorV3(hwMap, "senzor");
        colorSensor.getColorSensor().setGain(2);
    }

    public int[] getRGBColor() {
        return colorSensor.getARGB();
    }

    public float[] getHSVColor() {
        float[] HSVColors = new float[3];
        colorSensor.RGBtoHSV(colorSensor.red(), colorSensor.green(), colorSensor.blue(), HSVColors);

        return HSVColors;
    }

    public int getColor() { // GREEN = 1    PURPLE = 2
        float[] HSVColor = getHSVColor();
        if (HSVColor[0] > 100 && HSVColor[0] < 180) { /// 80 si 140
            return 1; // GREEN
        }
        if (HSVColor[0] > 180 && HSVColor[0] < 270){ /// 220 si 330
            return 2; // PURPLE
        }
        return 0;
    }

    public double getDistanceMM() {
        return colorSensor.distance(DistanceUnit.MM);
    }
}
