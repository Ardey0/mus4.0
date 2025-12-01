package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.SensorRevColorV3;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IntakeSubsystem extends SubsystemBase {
    private final Motor intake;

    public IntakeSubsystem(HardwareMap hwMap) {
        this.intake = new Motor(hwMap, "intake");
        this.intake.setInverted(true);
        this.intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.intake.setRunMode(Motor.RunMode.RawPower);
    }

    public void suck() {
        intake.set(1);
    }

    public void stop() {
        intake.set(0);
    }
}

