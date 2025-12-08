package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class ChassisDrive extends CommandBase {
    private final DriveSubsystem chassis;
    private final GamepadEx gamepad;

    public ChassisDrive(DriveSubsystem driveSubsystem, GamepadEx gamepad) {
        this.chassis = driveSubsystem;
        this.gamepad = gamepad;

        addRequirements(chassis);
    }

    public void execute() {
        chassis.drive(gamepad.getLeftX() * 1.1, gamepad.getLeftY(), gamepad.getRightX());
    }

}
