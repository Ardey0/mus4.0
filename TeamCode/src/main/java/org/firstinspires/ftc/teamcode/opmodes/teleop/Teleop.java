package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.IntakeBall;
import org.firstinspires.ftc.teamcode.commands.LaunchAllBalls;
import org.firstinspires.ftc.teamcode.commands.LaunchMotifBalls;
import org.firstinspires.ftc.teamcode.commands.LaunchBallByColor;
import org.firstinspires.ftc.teamcode.commands.LaunchBallBySector;
import org.firstinspires.ftc.teamcode.commands.PedroDrive;
import org.firstinspires.ftc.teamcode.commands.ReadMotif;
import org.firstinspires.ftc.teamcode.commands.SpitBalls;
import org.firstinspires.ftc.teamcode.commands.TrackAprilTag;
import org.firstinspires.ftc.teamcode.commands.UpdatePose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SenzorGauraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SenzorTavanSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OnofreiSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PaleteSubsytem;
import org.firstinspires.ftc.teamcode.subsystems.RobotStorage;

@TeleOp
public class Teleop extends CommandOpMode {
    private final double triggerMultiplier = 0.00134;
    private final Pose start = new Pose(55.700, 8.740, Math.toRadians(180));

    private TelemetryManager telemetryM;
    private GamepadEx gamepad;
    private Follower follower;
    private GoBildaPinpointDriver pinpoint;

    private DriveSubsystem chassis;
    private LauncherSubsystem launcher;
    private PaleteSubsytem palete;
    private OnofreiSubsystem onofrei;
    private IntakeSubsystem intake;
    private SenzorTavanSubsystem senzorTavan;
    private SenzorGauraSubsystem senzorGaura;
    private RobotStorage robotStorage;
    private LimelightSubsystem limelight;

    private Button intakeButton, launchMotifButton, readMotifButton, launchSector0Button,
            launchSector1Button, launchSector2Button, launchPurpleButton, launchGreenButton, launchAllButton,
            trackAprilTagButton, spitButton, updatePoseButton;

    @Override
    public void initialize() {
        super.reset();
        CommandScheduler.getInstance().setBulkReading(hardwareMap, LynxModule.BulkCachingMode.MANUAL);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        gamepad = new GamepadEx(gamepad1);
        follower = Constants.createFollower(hardwareMap);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        follower.setStartingPose(start);

        // Subsystems
        {
            chassis = new DriveSubsystem(hardwareMap);
            launcher = new LauncherSubsystem(hardwareMap);
            palete = new PaleteSubsytem(hardwareMap);
            onofrei = new OnofreiSubsystem(hardwareMap);
            intake = new IntakeSubsystem(hardwareMap);
            senzorTavan = new SenzorTavanSubsystem(hardwareMap);
            senzorGaura = new SenzorGauraSubsystem(hardwareMap);
            limelight = new LimelightSubsystem(hardwareMap, LimelightSubsystem.BLUE_APRILTAG_PIPELINE);
            robotStorage = new RobotStorage();
        }

        // Buttons
        {
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
            spitButton = new GamepadButton(
                    gamepad, GamepadKeys.Button.OPTIONS
            );
            updatePoseButton = new GamepadButton(
                    gamepad, GamepadKeys.Button.TOUCHPAD
            );
        }

//        chassis.setDefaultCommand(new ChassisDrive(chassis, gamepad));
        schedule(new PedroDrive(telemetryM, gamepad, follower));

        readMotifButton.whenPressed(new ReadMotif(robotStorage, telemetryM, limelight));

        intakeButton.toggleWhenPressed(new IntakeBall(robotStorage, telemetryM, intake, palete, senzorTavan, senzorGaura));

        updatePoseButton.whenPressed(new UpdatePose(follower, pinpoint, limelight));

        limelight.setDefaultCommand(new RunCommand(
                () -> {
                    telemetryM.addData("dist", limelight.getDistanceToDepot());
                }, limelight
        ));

        palete.setDefaultCommand(new RunCommand(
                () -> {
                    palete.setPosition(palete.getTargetPosition() - gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) * triggerMultiplier
                            + gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) * triggerMultiplier);
                    telemetryM.addData("palete pos", palete.getTargetPosition());
                }, palete
        ));

        trackAprilTagButton.toggleWhenPressed(new TrackAprilTag(telemetryM, gamepad, chassis, limelight));

        spitButton.toggleWhenPressed(new SpitBalls(intake));

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
        follower.update();
        telemetryM.update(telemetry);
    }
}
