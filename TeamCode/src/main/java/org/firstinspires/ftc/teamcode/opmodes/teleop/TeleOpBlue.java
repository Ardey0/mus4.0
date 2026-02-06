package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.Init;
import org.firstinspires.ftc.teamcode.commands.IntakeBallIndexing;
import org.firstinspires.ftc.teamcode.commands.LaunchAllBalls;
import org.firstinspires.ftc.teamcode.commands.LaunchMotifBalls;
import org.firstinspires.ftc.teamcode.commands.LaunchBallBySector;
import org.firstinspires.ftc.teamcode.commands.PedroDrive;
import org.firstinspires.ftc.teamcode.commands.ReadMotif;
import org.firstinspires.ftc.teamcode.commands.SpitBalls;
import org.firstinspires.ftc.teamcode.commands.TurnToGoalBlue;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeKickerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RampaSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SenzorGauraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SenzorRoataSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SenzorTavanSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OnofreiSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PaleteSubsytem;
import org.firstinspires.ftc.teamcode.subsystems.RobotStorage;

@Configurable
@TeleOp
public class TeleOpBlue extends CommandOpMode {
    public static double launcherSpeed = 0, rampAngle = 0;
    private final double triggerMultiplier = 0.006;
    private final int ALLIANCE = 0; // BLUE

    private TelemetryManager telemetryM;
    private GamepadEx gamepad;
    private Follower follower;
    private final ElapsedTime loopTime = new ElapsedTime();

    private LauncherSubsystem launcher;
    private PaleteSubsytem palete;
    private OnofreiSubsystem onofrei;
    private IntakeSubsystem intake;
    private IntakeKickerSubsystem intakeKicker;
    private RampaSubsystem rampa;
    private SenzorTavanSubsystem senzorTavan;
    private SenzorRoataSubsystem senzorRoata;
    private SenzorGauraSubsystem senzorGaura;
    private RobotStorage robotStorage;
    private LimelightSubsystem limelight;

    private Button intakeButton, launchMotifButton, readMotifButton, launchSector0Button,
            launchSector1Button, launchSector2Button, launchPurpleButton, launchGreenButton, launchAllButton,
            trackAprilTagButton, spitButton, resetRoataButton, kickerButton, resetHeadingButton;

    @Override
    public void initialize() {
        super.reset();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        gamepad = new GamepadEx(gamepad1);
        follower = Constants.createFollower(hardwareMap);

        // Subsystems
        {
            launcher = new LauncherSubsystem(hardwareMap);
            palete = new PaleteSubsytem(hardwareMap);
            onofrei = new OnofreiSubsystem(hardwareMap);
            intake = new IntakeSubsystem(hardwareMap);
            intakeKicker = new IntakeKickerSubsystem(hardwareMap);
            rampa = new RampaSubsystem(hardwareMap);
            senzorTavan = new SenzorTavanSubsystem(hardwareMap);
            senzorRoata = new SenzorRoataSubsystem(hardwareMap);
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
//            launchPurpleButton = new GamepadButton(
//                    gamepad, GamepadKeys.Button.LEFT_BUMPER
//            );
//            launchGreenButton = new GamepadButton(
//                    gamepad, GamepadKeys.Button.RIGHT_BUMPER
//            );
            spitButton = new GamepadButton(
                    gamepad, GamepadKeys.Button.OPTIONS
            );
            resetRoataButton = new GamepadButton(
                    gamepad, GamepadKeys.Button.TOUCHPAD
            );
            kickerButton = new GamepadButton(
                    gamepad, GamepadKeys.Button.LEFT_BUMPER
            );
            resetHeadingButton = new GamepadButton(
                    gamepad, GamepadKeys.Button.RIGHT_BUMPER
            );
        }

        Pose start = robotStorage.getAutoEndPose() == null ? new Pose(55.700, 8.740, Math.toRadians(180)) : robotStorage.getAutoEndPose();
        follower.setStartingPose(start);

        CommandScheduler.getInstance().setBulkReading(hardwareMap, LynxModule.BulkCachingMode.MANUAL);
        schedule(new Init(palete, onofrei, rampa, intakeKicker));

        schedule(new PedroDrive(telemetryM, gamepad, follower));
        readMotifButton.whenPressed(new ReadMotif(robotStorage, telemetryM, limelight));

        intakeButton.toggleWhenPressed(new IntakeBallIndexing(robotStorage, telemetryM, intake, palete, senzorTavan, senzorRoata, senzorGaura, intakeKicker));

        palete.setDefaultCommand(new RunCommand(
                () -> {
                    palete.setPosition(palete.getTargetPosition() - gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) * triggerMultiplier
                            + gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) * triggerMultiplier);
                    telemetryM.addData("palete pos", palete.getTargetPosition());
                }, palete
        ));

        resetRoataButton.whenPressed(new InstantCommand(
                () -> {
                    for (int i = 0; i < 3; i++)
                        robotStorage.setSector(i, 0);
                }
        ));

        trackAprilTagButton.toggleWhenPressed(new TurnToGoalBlue(follower));

        spitButton.whenPressed(new SpitBalls(intake));

        intakeKicker.setDefaultCommand(new RunCommand(
                () -> {
                    intakeKicker.setPosition(gamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER) ? IntakeKickerSubsystem.OUT : IntakeKickerSubsystem.IN);
                }, intakeKicker
        ));

        resetHeadingButton.whenPressed(new InstantCommand(
                () -> follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), Math.toRadians(180)))
        ));

        launchMotifButton.toggleWhenPressed(new LaunchMotifBalls(robotStorage, telemetryM, follower, palete, onofrei, launcher, rampa, ALLIANCE));
        launchAllButton.toggleWhenPressed(new LaunchAllBalls(robotStorage, telemetryM, follower, palete, onofrei, launcher, rampa, ALLIANCE));
        launchSector0Button.toggleWhenPressed(new LaunchBallBySector(robotStorage, telemetryM, follower, palete, onofrei, launcher, rampa, ALLIANCE, 0));
        launchSector1Button.toggleWhenPressed(new LaunchBallBySector(robotStorage, telemetryM, follower, palete, onofrei, launcher, rampa, ALLIANCE, 1));
        launchSector2Button.toggleWhenPressed(new LaunchBallBySector(robotStorage, telemetryM, follower, palete, onofrei, launcher, rampa, ALLIANCE, 2));
//        launchPurpleButton.toggleWhenPressed(new LaunchBallByColor(robotStorage, telemetryM, follower, palete, onofrei, launcher, rampa, ALLIANCE, 2));
//        launchGreenButton.toggleWhenPressed(new LaunchBallByColor(robotStorage, telemetryM, follower, palete, onofrei, launcher, rampa, ALLIANCE, 1));

//        launchSector0Button.toggleWhenPressed(new LaunchBallBySector(robotStorage, telemetryM, palete, onofrei, launcher, rampa, () -> launcherSpeed, () -> rampAngle, 0, 0));
    }

    @Override
    public void run() {
        super.run();

        follower.update();

        telemetryM.addData("dist to blue goal (m)", Math.sqrt((-follower.getPose().getX()) * (-follower.getPose().getX()) +
                (144 - follower.getPose().getY()) * (144 - follower.getPose().getY())) / 39.37007874);
        telemetryM.addData("heading error", Math.toDegrees(follower.getHeadingError()));
        telemetryM.addData("following path", follower.isBusy());
        telemetryM.addData("motif", robotStorage.getMotif()[0]);
        telemetryM.addData("loop time", loopTime.milliseconds());
        telemetryM.update(telemetry);

        loopTime.reset();
    }

    @Override
    public void end() {
        robotStorage.updateAutoEndPose(new Pose(55.700, 8.740, Math.toRadians(180)));
    }
}
