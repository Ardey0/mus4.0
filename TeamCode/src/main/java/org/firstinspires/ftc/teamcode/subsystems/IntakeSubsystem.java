package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

public class IntakeSubsystem extends SubsystemBase {
    private final Motor intake;

    public IntakeSubsystem(HardwareMap hwMap) {
        this.intake = new Motor(hwMap, "intake");
        this.intake.setInverted(true);
        this.intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.intake.setRunMode(Motor.RunMode.RawPower);
    }

    public void suck(double power) {
        intake.set(power);
    }

    public void spit() {
        intake.set(-0.6);
    }

    public void stop() {
        intake.set(0);
    }
}
