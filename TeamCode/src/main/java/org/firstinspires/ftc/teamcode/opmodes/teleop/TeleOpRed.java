package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
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
import org.firstinspires.ftc.teamcode.commands.TrackAprilTag;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OnofreiSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PaleteSubsytem;
import org.firstinspires.ftc.teamcode.subsystems.RobotStorage;

@TeleOp
@Disabled
public class TeleOpRed extends CommandOpMode {
    private TelemetryManager telemetryM;
    private GamepadEx gamepad;

    private DriveSubsystem chassis;
    private LauncherSubsystem launcher;
    private PaleteSubsytem palete;
    private OnofreiSubsystem onofrei;
    private IntakeSubsystem intake;
    private ColorSensorSubsystem sensor;
    private RobotStorage robotStorage;
    private LimelightSubsystem limelight;

    private Button intakeButton, launchMotifButton, readMotifButton, launchSector0Button,
            launchSector1Button, launchSector2Button, launchPurpleButton, launchGreenButton, launchAllButton,
            setLaunchDistanceFarButton, setLaunchDistanceNearButton, trackAprilTagButton;

    private boolean launchFromFar = false; // cititi asta ca pe o intrebare

    @Override
    public void initialize() {
        super.reset();
        CommandScheduler.getInstance().setBulkReading(hardwareMap, LynxModule.BulkCachingMode.MANUAL);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        gamepad = new GamepadEx(gamepad1);

        chassis = new DriveSubsystem(hardwareMap);
        launcher = new LauncherSubsystem(hardwareMap);
        palete = new PaleteSubsytem(hardwareMap);
        onofrei = new OnofreiSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        sensor = new ColorSensorSubsystem(hardwareMap);
        limelight = new LimelightSubsystem(hardwareMap, LimelightSubsystem.RED_APRILTAG_PIPELINE);
        robotStorage = new RobotStorage();


        intakeButton = new GamepadButton(
                gamepad, GamepadKeys.Button.CROSS
        );
        launchMotifButton = new GamepadButton(
                gamepad, GamepadKeys.Button.CIRCLE
        );
        readMotifButton = new GamepadButton(
                gamepad, GamepadKeys.Button.SHARE
        );
        trackAprilTagButton = new GamepadButton(
                gamepad, GamepadKeys.Button.TRIANGLE
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


        chassis.setDefaultCommand(new ChassisDrive(chassis, gamepad));

        readMotifButton.toggleWhenPressed(new ReadMotif(robotStorage, telemetryM, limelight));

        intakeButton.toggleWhenPressed(new IntakeBall(robotStorage, telemetryM, intake, palete, sensor));

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
        trackAprilTagButton.toggleWhenPressed(new TrackAprilTag(gamepad, telemetryM, chassis, limelight));

        launchMotifButton.toggleWhenPressed(new LaunchMotifBalls(robotStorage, telemetryM, palete, onofrei, launcher, limelight));
        launchAllButton.toggleWhenPressed(new LaunchAllBalls(robotStorage, telemetryM, palete, onofrei, launcher, limelight));
        launchSector0Button.toggleWhenPressed(new LaunchBallBySector(robotStorage, telemetryM, palete, onofrei, launcher, limelight, 0));
        launchSector1Button.toggleWhenPressed(new LaunchBallBySector(robotStorage, telemetryM, palete, onofrei, launcher, limelight, 1));
        launchSector2Button.toggleWhenPressed(new LaunchBallBySector(robotStorage, telemetryM, palete, onofrei, launcher, limelight, 2));
        launchPurpleButton.toggleWhenPressed(new LaunchBallByColor(robotStorage, telemetryM, palete, onofrei, launcher, limelight, 2));
        launchGreenButton.toggleWhenPressed(new LaunchBallByColor(robotStorage, telemetryM, palete, onofrei, launcher, limelight, 1));
    }

    @Override
    public void run() {
        super.run();
        telemetryM.addData("launcher target", launchFromFar ? "departe" : "aproape");
        telemetryM.update(telemetry);
    }
}
