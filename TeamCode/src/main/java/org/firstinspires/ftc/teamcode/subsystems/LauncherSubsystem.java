package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

public class LauncherSubsystem extends SubsystemBase {
    private final double kP = 0.0054, kI = 0, kD = 0.0000015, kF = 0.000335;
    private final MotorEx flywheel;
    private final PIDFController controller = new PIDFController(kP, kI, kD, kF);

    public LauncherSubsystem(HardwareMap hwMap) {
        this.flywheel = new MotorEx(hwMap, "launcher", 112, 6000);
        this.flywheel.setInverted(true);
        this.flywheel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        this.flywheel.setRunMode(Motor.RunMode.RawPower);
        controller.setTolerance(11);
    }

    public boolean atTargetSpeed() {
        return controller.atSetPoint();
    }

    public double getVelocity() {
        return flywheel.getVelocity();
    }

    public void spin(double speed) {
        if (atTargetSpeed()) {
            controller.setPIDF(0, 0, 0, kF);
        } else {
            controller.setPIDF(kP, kI, kD, kF);
        }
        double power = controller.calculate(flywheel.getVelocity(), speed);
        flywheel.set(power);
    }

    public void stop() {
        flywheel.set(0);
    }

    public void brake() {
        this.flywheel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }
}
