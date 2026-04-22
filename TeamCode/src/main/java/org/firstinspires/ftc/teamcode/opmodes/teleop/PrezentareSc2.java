package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.Init;
import org.firstinspires.ftc.teamcode.commands.Intake;
import org.firstinspires.ftc.teamcode.commands.LaunchAll;
import org.firstinspires.ftc.teamcode.commands.PedroDriveFake;
import org.firstinspires.ftc.teamcode.commands.SpitBalls;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeKickerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OnofreiSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PaleteSubsytem;
import org.firstinspires.ftc.teamcode.subsystems.RampaSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotStorage;
import org.firstinspires.ftc.teamcode.subsystems.SenzorGauraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SenzorRoataSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SenzorTavanSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TiltSubsystem;

@TeleOp
public class PrezentareSc2 extends CommandOpMode {
    private TelemetryManager telemetryM;
    private GamepadEx gamepad;
    private Follower follower;

    private final ElapsedTime loopTime = new ElapsedTime();
    private final ElapsedTime gameTime = new ElapsedTime();

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
    private TiltSubsystem tilt;

    private Button intakeButton, launchAllButton, spitButton;

    @Override
    public void initialize() {
        super.reset();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        gamepad = new GamepadEx(gamepad1);
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(0.5);
        gameTime.reset();

        launcher = new LauncherSubsystem(hardwareMap);
        palete = new PaleteSubsytem(hardwareMap);
        onofrei = new OnofreiSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        intakeKicker = new IntakeKickerSubsystem(hardwareMap);
        rampa = new RampaSubsystem(hardwareMap);
        senzorTavan = new SenzorTavanSubsystem(hardwareMap);
        senzorRoata = new SenzorRoataSubsystem(hardwareMap);
        senzorGaura = new SenzorGauraSubsystem(hardwareMap);
        robotStorage = new RobotStorage();
        tilt = new TiltSubsystem(hardwareMap);

        intakeButton = new GamepadButton(
                gamepad, GamepadKeys.Button.CROSS
        );
        launchAllButton = new GamepadButton(
                gamepad, GamepadKeys.Button.DPAD_UP
        );
        spitButton = new GamepadButton(
                gamepad, GamepadKeys.Button.SQUARE
        );

        Pose start = new Pose(0, 0, Math.toRadians(180));
        follower.setStartingPose(start);

        CommandScheduler.getInstance().setBulkReading(hardwareMap, LynxModule.BulkCachingMode.MANUAL);
        schedule(new Init(palete, onofrei, rampa, intakeKicker, tilt));

        schedule(new PedroDriveFake(telemetryM, gamepad, follower));
        launchAllButton.toggleWhenPressed(new LaunchAll(robotStorage, telemetryM, palete, onofrei, launcher, rampa, 1250, 0.2, 1));
        intakeButton.toggleWhenPressed(new Intake(robotStorage, telemetryM, intake, palete, senzorTavan, senzorRoata, senzorGaura, intakeKicker));
        spitButton.whenPressed(new SpitBalls(intake));
    }

    @Override
    public void run() {
        super.run();

        telemetryM.addData("loop time", loopTime.milliseconds());
        telemetryM.addData("game time", gameTime.seconds());
        telemetryM.addData("artifact count", robotStorage.getBallCount());

        telemetryM.update(telemetry);
        follower.update();
        loopTime.reset();
    }
}
