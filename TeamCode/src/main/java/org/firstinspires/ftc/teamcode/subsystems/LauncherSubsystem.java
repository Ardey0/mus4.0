package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

public class LauncherSubsystem extends SubsystemBase {
    private final Motor flywheel;

    public LauncherSubsystem(Motor flywheel) {
        this.flywheel = flywheel;
        this.flywheel.setInverted(true);
        this.flywheel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.flywheel.setRunMode(Motor.RunMode.RawPower);
    }

    public void spin() {
        flywheel.set(1);
    }

    public void stop() {
        flywheel.set(0);
    }
}
