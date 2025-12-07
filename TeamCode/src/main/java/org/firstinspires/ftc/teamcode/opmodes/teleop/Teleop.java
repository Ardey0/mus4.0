package org.firstinspires.ftc.teamcode.OpModes.TeleOP;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.ChassisDrive;
import org.firstinspires.ftc.teamcode.commands.IntakeBall;
import org.firstinspires.ftc.teamcode.commands.LaunchAllBalls;
import org.firstinspires.ftc.teamcode.commands.LaunchMotifBalls;
import org.firstinspires.ftc.teamcode.commands.LaunchBallByColor;
import org.firstinspires.ftc.teamcode.commands.LaunchBallBySector;
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

    private Button intakeButton, launchMotifButton, scanMotifButton, launchSector0Button,
            launchSector1Button, launchSector2Button, launchPurpleButton, launchGreenButton, launchAllButton,
            setLaunchDistanceFarButton, setLaunchDistanceNearButton;

    private boolean launchFromFar = false; // cititi asta ca pe o intrebare

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

        intakeButton = new GamepadButton(
                gamepad, GamepadKeys.Button.CROSS
        );
        launchMotifButton = new GamepadButton(
                gamepad, GamepadKeys.Button.CIRCLE
        );
        scanMotifButton = new GamepadButton(
                gamepad, GamepadKeys.Button.SHARE
        );
        launchSector0Button = new GamepadButton(
                gamepad, GamepadKeys.Button.DPAD_RIGHT
        );
        launchSector1Button = new GamepadButton(
                gamepad, GamepadKeys.Button.DPAD_DOWN
        );
        launchSector2Button = new GamepadButton(
                gamepad, GamepadKeys.Button.DPAD_LEFT
        );
        launchAllButton = new GamepadButton(
                gamepad, GamepadKeys.Button.DPAD_UP
        );
        launchPurpleButton = new GamepadButton(
                gamepad, GamepadKeys.Button.LEFT_BUMPER
        );
        launchGreenButton = new GamepadButton(
                gamepad, GamepadKeys.Button.RIGHT_BUMPER
        );
        setLaunchDistanceFarButton = new GamepadButton(
                gamepad, GamepadKeys.Button.RIGHT_STICK_BUTTON
        );
        setLaunchDistanceNearButton = new GamepadButton(
                gamepad, GamepadKeys.Button.LEFT_STICK_BUTTON
        );


        schedule(drive);

        scanMotifButton.toggleWhenPressed(new ReadMotif(robotStorage, telemetry, limelight));

        intakeButton.toggleWhenPressed(new IntakeBall(robotStorage, telemetry, intake, palete, sensor));

        setLaunchDistanceFarButton.whenPressed(() -> {
            launchFromFar = true;
            telemetry.addLine("DEPARTE");
            telemetry.update();
        });
        setLaunchDistanceNearButton.whenPressed(() -> {
            launchFromFar = false;
            telemetry.addLine("APROAPE");
            telemetry.update();
        });
        launchMotifButton.toggleWhenPressed(new LaunchMotifBalls(robotStorage, telemetry, palete, onofrei, launcher, () -> launchFromFar));
        launchAllButton.toggleWhenPressed(new LaunchAllBalls(robotStorage, telemetry, palete, onofrei, launcher, () -> launchFromFar));
        launchSector0Button.toggleWhenPressed(new LaunchBallBySector(robotStorage, telemetry, palete, onofrei, launcher, () -> launchFromFar, 0));
        launchSector1Button.toggleWhenPressed(new LaunchBallBySector(robotStorage, telemetry, palete, onofrei, launcher, () -> launchFromFar, 1));
        launchSector2Button.toggleWhenPressed(new LaunchBallBySector(robotStorage, telemetry, palete, onofrei, launcher, () -> launchFromFar, 2));
        launchPurpleButton.toggleWhenPressed(new LaunchBallByColor(robotStorage, telemetry, palete, onofrei, launcher, () -> launchFromFar, 2));
        launchGreenButton.toggleWhenPressed(new LaunchBallByColor(robotStorage, telemetry, palete, onofrei, launcher, () -> launchFromFar, 1));
    }
}
