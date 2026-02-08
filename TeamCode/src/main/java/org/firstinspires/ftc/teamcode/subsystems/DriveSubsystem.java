package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

public class DriveSubsystem extends SubsystemBase {
    private final Motor frontLeft, frontRight, backLeft, backRight;

    public DriveSubsystem(HardwareMap hwMap) {
        frontLeft = new Motor(hwMap, "front_left");
        frontRight = new Motor(hwMap, "front_right");
        backLeft = new Motor(hwMap, "back_left");
        backRight = new Motor(hwMap, "back_right");

        backLeft.setInverted(true);
        backRight.setInverted(true);
        frontRight.setInverted(true);

        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void drive(double x, double y, double r) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(r), 1);
        double frontLeftPower = (y + x + r) / denominator;
        double backLeftPower = (y - x + r) / denominator;
        double frontRightPower = (y - x - r) / denominator;
        double backRightPower = (y + x - r) / denominator;

        frontLeft.set(frontLeftPower);
        backLeft.set(backLeftPower);
        frontRight.set(frontRightPower);
        backRight.set(backRightPower);
    }
}
