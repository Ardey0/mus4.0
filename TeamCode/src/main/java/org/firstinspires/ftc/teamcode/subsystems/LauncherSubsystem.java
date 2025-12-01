package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

public class LauncherSubsystem extends SubsystemBase {
    private final double kP = 0.004, kI = 0, kD = 0.0000007, kF = 0.000375, targetSpeed = 1450;
    private final MotorEx flywheel;
    private final PIDFController controller = new PIDFController(kP, kI, kD, kF);

    public LauncherSubsystem(HardwareMap hwMap) {
        this.flywheel = new MotorEx(hwMap, "launcher");
        this.flywheel.setInverted(true);
        this.flywheel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.flywheel.setRunMode(Motor.RunMode.RawPower);
    }

    public void spin() {
        double power = controller.calculate(flywheel.getVelocity(), targetSpeed);
        flywheel.set(power);
    }

    public void stop() {
        flywheel.set(0);
    }
}
