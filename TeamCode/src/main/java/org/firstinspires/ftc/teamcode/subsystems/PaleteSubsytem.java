package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

public class PaleteSubsytem extends SubsystemBase {
    private final ServoEx palete;
    public static final double IN_BILA_1 = 0.58,
                               IN_BILA_2 = 0.92,
                               IN_BILA_3 = 0.235,
                               OUT_BILA_1 = 0.78,
                               OUT_BILA_2 = 0.08,
                               OUT_BILA_3 = 0.43,
                               LOCK = 0.3;

    public PaleteSubsytem(HardwareMap hwMap) {
        this.palete = new ServoEx(hwMap, "palete");
        // power catching
    }

    public void setPosition(double position) {
        palete.set(position);
    }
}
