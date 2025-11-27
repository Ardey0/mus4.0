package org.firstinspires.ftc.teamcode.OpModes;

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
import org.firstinspires.ftc.teamcode.commands.ReadMotif;
import org.firstinspires.ftc.teamcode.subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OnofreiSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PaleteSubsytem;
import org.firstinspires.ftc.teamcode.subsystems.RoataSubsystem;

@TeleOp
public class Teleop extends CommandOpMode {
    private GamepadEx gamepad;
    private ChassisSubsystem chassis;
    private LauncherSubsystem launcher;
    private PaleteSubsytem palete;
    private OnofreiSubsystem onofrei;
    private IntakeSubsystem intake;
    private ColorSensorSubsystem sensor;
    private RoataSubsystem roata;
    private LimelightSubsystem limelight;
    private ChassisDrive drive;
    private IntakeBall intakeBall;
    private LaunchBall launchBall;
    private ReadMotif readMotif;

    private Button intakeButton, launcherButton, scanMotifButton;

    @Override
    public void initialize() {
        gamepad = new GamepadEx(gamepad1);

        chassis = new ChassisSubsystem(hardwareMap);
        launcher = new LauncherSubsystem(hardwareMap);
        palete = new PaleteSubsytem(hardwareMap);
        onofrei = new OnofreiSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        sensor = new ColorSensorSubsystem(hardwareMap);
        limelight = new LimelightSubsystem(hardwareMap);
        roata = new RoataSubsystem();

        drive = new ChassisDrive(chassis, gamepad);
        readMotif = new ReadMotif(limelight);
        intakeBall = new IntakeBall(intake, palete, sensor, roata, telemetry);
        launchBall = new LaunchBall(roata, palete, onofrei, launcher, readMotif.tagId);


        intakeButton = new GamepadButton(
                gamepad, GamepadKeys.Button.CROSS
        );
        launcherButton = new GamepadButton(
                gamepad, GamepadKeys.Button.CIRCLE
        );
        scanMotifButton = new GamepadButton(
                gamepad, GamepadKeys.Button.SHARE
        );

        schedule(drive);
        scanMotifButton.toggleWhenPressed(readMotif);
        intakeButton.toggleWhenPressed(intakeBall);
        launcherButton.toggleWhenPressed(launchBall);
    }
}
