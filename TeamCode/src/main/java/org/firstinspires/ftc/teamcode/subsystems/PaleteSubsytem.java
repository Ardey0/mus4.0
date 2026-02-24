package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

public class PaleteSubsytem extends SubsystemBase {
    private final ServoEx palete;
    public static final double IN_BILA_0 = 0.935,
                               IN_BILA_1 = 0.247,
                               IN_BILA_2 = 0.591,
                               OUT_BILA_0 = 0.097,
                               OUT_BILA_1 = 0.44,
                               OUT_BILA_2 = 0.785,
                               LOCK = 0.3;

    public PaleteSubsytem(HardwareMap hwMap) {
        this.palete = new ServoEx(hwMap, "palete");
        this.palete.setCachingTolerance(0.0005);
    }

    public void setPosition(double position) {
        palete.set(position);
    }

    public double getTargetPosition() {
        return palete.getRawPosition();
    }
}