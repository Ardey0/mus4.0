package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.ChassisSubsystem;

public class ChassisDrive extends CommandBase {
    private final ChassisSubsystem chassis;
    private final GamepadEx gamepad;

    public ChassisDrive(ChassisSubsystem chassisSubsystem, GamepadEx gamepad) {
        this.chassis = chassisSubsystem;
        this.gamepad = gamepad;

        addRequirements(chassis);
    }

    public void execute() {
        chassis.drive(gamepad.getLeftX() * 1.1, gamepad.getLeftY(), gamepad.getRightX());
    }

}
