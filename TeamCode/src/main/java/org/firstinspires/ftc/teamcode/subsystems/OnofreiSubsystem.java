package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

@Configurable
public class OnofreiSubsystem extends SubsystemBase {
    private final ServoEx onofrei;
    public static double OUT = 1,
                               FAN_FIRE = 0.42,
                               IN = 0;

    public OnofreiSubsystem(HardwareMap hwMap) {
        this.onofrei = new ServoEx(hwMap, "onofrei");
    }

    public void setPosition(double position) {
        onofrei.set(position);
    }
}
