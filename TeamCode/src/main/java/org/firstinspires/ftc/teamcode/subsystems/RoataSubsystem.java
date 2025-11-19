package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

public class RoataSubsystem extends SubsystemBase {
    private final ServoEx palete, onofrei;

    public RoataSubsystem(HardwareMap hwMap) {
        this.palete = new ServoEx(hwMap, "palete");
        this.onofrei = new ServoEx(hwMap, "onofrei");
        /// power catching
    }

    public void setPaletePosition(double pos) {
        palete.set(pos);
    }

    public void setOnofreiPosition(double pos) {
        onofrei.set(pos);
    }
}
