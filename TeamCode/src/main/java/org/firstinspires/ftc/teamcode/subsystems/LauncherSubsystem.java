package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

public class LauncherSubsystem extends SubsystemBase {
    private final double kP = 0.004, kI = 0, kD = 0.0000015, kF = 0.00035;
    private final MotorEx flywheel1, flywheel2;
    private final PIDFController controller = new PIDFController(kP, kI, kD, kF);

    public LauncherSubsystem(HardwareMap hwMap) {
        this.flywheel1 = new MotorEx(hwMap, "launcher");
        this.flywheel2 = new MotorEx(hwMap, "launcher2");
        this.flywheel1.setInverted(true);
        this.flywheel1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        this.flywheel1.setRunMode(Motor.RunMode.RawPower);
        this.flywheel2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        this.flywheel2.setRunMode(Motor.RunMode.RawPower);
        controller.setTolerance(11);
    }

    public boolean atTargetSpeed() {
        return controller.atSetPoint();
    }

    public double getVelocity() {
        return flywheel1.getVelocity();
    }

    public void spin(double speed) {
        if (atTargetSpeed()) {
            controller.setPIDF(0, 0, 0, kF);
        } else {
            controller.setPIDF(kP, kI, kD, kF);
        }
        double power = controller.calculate(flywheel1.getVelocity(), speed);
        flywheel1.set(power);
    }

    public void stop() {
        flywheel1.set(0);
        flywheel2.set(0);
    }

    public void brake() {
        this.flywheel1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.flywheel2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }
}
