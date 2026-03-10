package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

public class TiltSubsystem extends SubsystemBase {
    private final ServoEx servo1, servo2;
    public static double UP = 0.5,
                         DOWN = 0;

    public TiltSubsystem(HardwareMap hwMap) {
        this.servo1 = new ServoEx(hwMap, "tilt1");
        this.servo2 = new ServoEx(hwMap, "tilt2");
        this.servo1.setCachingTolerance(0.01);
        this.servo2.setCachingTolerance(0.01);
    }

    public void setPosition(double position) {
        servo1.set(position);
        servo2.set(position);
    }
}
