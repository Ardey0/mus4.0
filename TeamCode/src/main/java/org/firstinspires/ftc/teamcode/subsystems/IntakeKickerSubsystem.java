package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

public class IntakeKickerSubsystem extends SubsystemBase {
    private final ServoEx kicker;
    public static final double OUT = 0.5,
            IN = 0;

    public IntakeKickerSubsystem(HardwareMap hwMap) {
        this.kicker = new ServoEx(hwMap, "servo_intake");
//        this.kicker.setCachingTolerance(0.0001);
    }

    public void setPosition(double position) {
        kicker.set(position);
    }
}
