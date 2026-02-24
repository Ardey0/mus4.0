package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import java.util.function.DoubleSupplier;

@Configurable
public class RampaSubsystem extends SubsystemBase {
    private final ServoEx rampa;
    public static final double OUT = 0.93,
                               IN = 0.01;

    public RampaSubsystem(HardwareMap hwMap) {
        this.rampa = new ServoEx(hwMap, "rampa");
        this.rampa.setCachingTolerance(0.01);
    }

    public void setPosition(double position) {
        rampa.set(position);
    }

    public void setPosition(DoubleSupplier position) {
        rampa.set(position.getAsDouble());
    }
}
