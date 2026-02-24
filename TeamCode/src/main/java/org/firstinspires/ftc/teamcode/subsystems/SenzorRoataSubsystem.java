package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.SensorRevColorV3;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class SenzorRoataSubsystem extends SubsystemBase {
    private final SensorRevColorV3 colorSensor;

    public SenzorRoataSubsystem(HardwareMap hwMap){
        this.colorSensor = new SensorRevColorV3(hwMap, "senzor_tavan");
        colorSensor.getColorSensor().setGain(2);
        ((LynxI2cDeviceSynch) colorSensor.getColorSensor().getDeviceClient()).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
    }

    public double getDistanceMM() {
        return colorSensor.distance(DistanceUnit.MM);
    }
}
