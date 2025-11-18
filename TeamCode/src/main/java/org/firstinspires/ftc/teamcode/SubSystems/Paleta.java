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
                COLLECT2 = 0.36,
                COLLECT3 = 0.71,
                LAUNCH1 = 0.85,
                LAUNCH2 = 0.14,
                LAUNCH3 = 0.5,
                BLOCAT = 1;
        public enum State{
            SHUFFLE,
            LAUNCH,
            COLLECT

        }
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

        public State getState(){
            State state;
            double angle = getPosition();
            return null;
        }
        public void disable() {
            if (servo.isPwmEnabled()) {
                servo.setPwmDisable();
           }
        }
}

