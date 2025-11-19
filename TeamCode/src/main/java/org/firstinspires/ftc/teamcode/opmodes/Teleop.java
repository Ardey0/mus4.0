package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.commands.ChassisDrive;
import org.firstinspires.ftc.teamcode.commands.IntakeBall;
import org.firstinspires.ftc.teamcode.commands.LaunchBall;
import org.firstinspires.ftc.teamcode.subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

@TeleOp
public class Teleop extends CommandOpMode {
    private GamepadEx gamepad;
    private Button intakeButton;

    @Override
    public void initialize() {
        gamepad = new GamepadEx(gamepad1);
        intakeButton = new GamepadButton(
                gamepad, GamepadKeys.Button.A
        );

        schedule(new ChassisDrive(new ChassisSubsystem(hardwareMap), gamepad));
        intakeButton.toggleWhenPressed(new IntakeBall(new IntakeSubsystem(hardwareMap)));
    }
}
