package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

@Configurable
public class LauncherSubsystem extends SubsystemBase {
    public static double kP = 0.002, kI = 0, kD = 0.00000002, kF = 0.00039;
    private final double nominalVoltage = 12;
    private final MotorEx flywheel1, flywheel2;
    private final VoltageSensor voltageSensor;
    private final PIDFController controller = new PIDFController(kP, kI, kD, kF);

    public LauncherSubsystem(HardwareMap hwMap) {
        this.flywheel1 = new MotorEx(hwMap, "launcher");
        this.flywheel2 = new MotorEx(hwMap, "launcher2");
        this.voltageSensor = hwMap.voltageSensor.iterator().next();
        this.flywheel1.setInverted(true);
        this.flywheel1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        this.flywheel1.setRunMode(Motor.RunMode.RawPower);
        this.flywheel1.setCachingTolerance(0.0001);
        this.flywheel2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        this.flywheel2.setRunMode(Motor.RunMode.RawPower);
        this.flywheel1.setCachingTolerance(0.0001);

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
        double power = controller.calculate(flywheel1.getVelocity(), speed) * getCompensationCoefficient();
        flywheel1.set(power);
        flywheel2.set(power);
    }

    public void stop() {
        flywheel1.stopMotor();
        flywheel2.stopMotor();
    }

    public void brake() {
        this.flywheel1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.flywheel2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    // voltage compensation
    private double getCompensationCoefficient() {
        return nominalVoltage / voltageSensor.getVoltage();
    }
}
