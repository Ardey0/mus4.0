package org.firstinspires.ftc.teamcode.SubSystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Paleta {
        private ServoImplEx servo = null;
        private AnalogInput encoder = null;
        public static final double
                COLLECT1 = 0,
                COLLECT2 = 0,
                COLLECT3 = 0,
                LANSEAZA1 = 0,
                LANSEAZA2 = 0,
                LANSEAZA3 = 0;

        public Paleta(@NonNull HardwareMap hardwareMap){
            this.servo = hardwareMap.get(ServoImplEx.class, "paleta");
            this.encoder = hardwareMap.get(AnalogInput.class, "encoder");
        }

        public void SetPosition(double position) {
            servo.setPosition(position);
        }
        public double getPosition() {
        return encoder.getVoltage() / 3.3 * 360;
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

