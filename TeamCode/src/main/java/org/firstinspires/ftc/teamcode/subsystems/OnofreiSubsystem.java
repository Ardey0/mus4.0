package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

public class OnofreiSubsystem extends SubsystemBase {
    private final ServoEx onofrei;
    public static final double OUT = 0.3,
                               IN = 0;

    public OnofreiSubsystem(HardwareMap hwMap) {
        this.onofrei = new ServoEx(hwMap, "onofrei");
        // power catching
    }

    public void setPosition(double position) {
        onofrei.set(position);
    }
}
