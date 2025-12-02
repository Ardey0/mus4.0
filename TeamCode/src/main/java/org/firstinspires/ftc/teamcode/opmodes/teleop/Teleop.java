package org.firstinspires.ftc.teamcode.OpModes.TeleOP;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

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
import org.firstinspires.ftc.teamcode.subsystems.RobotStorage;

@TeleOp
public class Teleop extends CommandOpMode {
    private final PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;

    private GamepadEx gamepad;

    private ChassisSubsystem chassis;
    private LauncherSubsystem launcher;
    private PaleteSubsytem palete;
    private OnofreiSubsystem onofrei;
    private IntakeSubsystem intake;
    private ColorSensorSubsystem sensor;
    private RobotStorage robotStorage;
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
        robotStorage = new RobotStorage();

        drive = new ChassisDrive(chassis, gamepad);
        readMotif = new ReadMotif(robotStorage, telemetry, limelight);
        intakeBall = new IntakeBall(robotStorage, telemetry, intake, palete, sensor);
        launchBall = new LaunchBall(robotStorage, telemetry, palete, onofrei, launcher);


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
//        schedule(new FunctionalCommand(() -> {panelsTelemetry.getTelemetry().update(telemetry);},
//                () -> {},
//                (isInterrupted) -> {},
//                () -> true
//        )); // nu sunt mandru de asta
        scanMotifButton.toggleWhenPressed(readMotif);
        intakeButton.toggleWhenPressed(intakeBall);
        launcherButton.toggleWhenPressed(new LaunchBall(robotStorage, telemetry, palete, onofrei, launcher));
    }
}
