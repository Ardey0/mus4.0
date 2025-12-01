package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.jetbrains.annotations.NotNull;


public class Onofrei {
    private Servo onofrei;
    private ServoImplEx servo = null;

    public static final double CLOSED=1, OPEN=0;
    public Onofrei(@NotNull HardwareMap hardwareMap) {
        this.servo = hardwareMap.get(ServoImplEx.class, "onofrei");
    }

    public void SetPositon (double position) {
        servo.setPosition(position);
    }
    public void enable() {
        if (!servo.isPwmEnabled()) {
            servo.setPwmEnable();
        }
    }
    public void disable() {
        if (servo.isPwmEnabled()) {
            servo.setPwmDisable();
                    }
            }
}
