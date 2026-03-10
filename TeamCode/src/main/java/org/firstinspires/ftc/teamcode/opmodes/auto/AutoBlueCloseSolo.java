package org.firstinspires.ftc.teamcode.opmodes.auto;


import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.pedroCommand.TurnToCommand;

import org.firstinspires.ftc.teamcode.commands.Init;
import org.firstinspires.ftc.teamcode.commands.Intake;
import org.firstinspires.ftc.teamcode.commands.LaunchFanFire;
import org.firstinspires.ftc.teamcode.commands.IntakeIndexing;
import org.firstinspires.ftc.teamcode.commands.LaunchAll;
import org.firstinspires.ftc.teamcode.commands.LaunchMotif;
import org.firstinspires.ftc.teamcode.commands.ReadMotif;
import org.firstinspires.ftc.teamcode.commands.SpitBalls;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeKickerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OnofreiSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PaleteSubsytem;
import org.firstinspires.ftc.teamcode.subsystems.RampaSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotStorage;
import org.firstinspires.ftc.teamcode.subsystems.SenzorGauraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SenzorRoataSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SenzorTavanSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TiltSubsystem;

@Autonomous
public class AutoBlueCloseSolo extends CommandOpMode {
    private final int ALLIANCE = 0; // BLUE

    private TelemetryManager telemetryM;

    private LauncherSubsystem launcher;
    private PaleteSubsytem palete;
    private OnofreiSubsystem onofrei;
    private IntakeSubsystem intake;
    private TiltSubsystem tilt;
    private IntakeKickerSubsystem intakeKicker;
    private RampaSubsystem rampa;
    private SenzorTavanSubsystem senzorTavan;
    private SenzorRoataSubsystem senzorRoata;
    private SenzorGauraSubsystem senzorGaura;
    private RobotStorage robotStorage;
    private LimelightSubsystem limelight;

    private Init init;

    private ElapsedTime loopTime;

    private Follower follower;


    public PathChain preload;
    private PathChain Exit;
    public PathChain Grab1;
    public PathChain Grab2;
    public PathChain Launch2;
    public PathChain Launch1;
    public PathChain Grab3;
    public PathChain Launch3;
    public PathChain ClearGate;
    public PathChain Motif;
    public PathChain LaunchGate;
    public PathChain GrabGate;
    private final Pose start = new Pose(19.26, 121.36, Math.toRadians(-127.6));
    private final Pose launchPre = new Pose(45, 95, Math.toRadians(-131));
    private final Pose motif = new Pose(45, 95, Math.toRadians(-180));
    private final Pose launch1 = new Pose(53, 90, Math.toRadians(-129));
    private final Pose launchEdge = new Pose(53, 75.5, Math.toRadians(-137));
    private final Pose launchEdge2 = new Pose(54, 76.5, Math.toRadians(-137));
    private final Pose launchFinal = new Pose(56, 98, Math.toRadians(-130));
    private final Pose grab1 = new Pose(22, 84.5, Math.toRadians(180));
    private final Pose clearGateSafe = new Pose(13, 73, Math.toRadians(90));
    private final Pose clearGateRisky = new Pose(13.7,64.5, Math.toRadians(180));
    private final Pose grab2 = new Pose(17, 58, Math.toRadians(180));
    private final Pose grab3 = new Pose(17, 35, Math.toRadians(180));
    private final Pose grabGate = new Pose(9.5, 60, Math.toRadians(146));
    private final Pose exit = new Pose(34, 75, Math.toRadians(-135));

    public void buildPaths() {
        preload = follower.pathBuilder()
                .addPath(
                        new BezierLine(start, launchPre)
                )
                .setLinearHeadingInterpolation(start.getHeading(), launchPre.getHeading())
                .build();

        Motif = follower.pathBuilder()
                .addPath(
                        new BezierPoint(motif)
                )
                .build();

        Grab1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                launchEdge2,
                                new Pose(48.8, 89.5),
                                grab1)
                )
                .setLinearHeadingInterpolation(launchEdge2.getHeading(),grab1.getHeading())
                .build();
        ClearGate = follower.pathBuilder()
                .addPath(
                        new BezierCurve(grab2,
                                new Pose(21.66, 61.83),
                                clearGateRisky)
                )
                .setConstantHeadingInterpolation(clearGateRisky.getHeading())
                .build();
        Launch1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(grab1, launchFinal)
                )
                .setLinearHeadingInterpolation(grab1.getHeading(), launchFinal.getHeading())
                .build();
        Grab2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                launchPre,
                                new Pose(47.981, 53.975),
                                new Pose(51.262, 61.897),
                                grab2
                        )
                )
                .addParametricCallback(0.6, () ->
                        follower.setMaxPower(0.5)
                )
                .addParametricCallback(1, () ->
                        follower.setMaxPower(1)
                )
                .setLinearHeadingInterpolation(launchPre.getHeading(), grab2.getHeading())
                .build();
        Launch2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                grab2,
                                launchEdge)
                )
                .addParametricCallback(0.7, () ->
                        follower.setMaxPower(0.5)
                )
                .addParametricCallback(1, () ->
                        follower.setMaxPower(1)
                )
                .setLinearHeadingInterpolation(clearGateRisky.getHeading(), launchEdge.getHeading())
                .build();
        GrabGate = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                launchEdge,
                                new Pose(30.79,60.4),
                                grabGate
                        )
                )
                .addParametricCallback(0.6, () ->
                        follower.setMaxPower(0.5)
                )
                .addParametricCallback(1, () ->
                        follower.setMaxPower(1)
                )
                .setLinearHeadingInterpolation(launchEdge.getHeading(),grabGate.getHeading())
                .build();
        LaunchGate = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                grabGate,
                                launchEdge2
                        )
                )
                .addParametricCallback(0.65, () ->
                        follower.setMaxPower(0.5)
                )
                .addParametricCallback(1, () ->
                        follower.setMaxPower(1)
                )
                .setLinearHeadingInterpolation(grabGate.getHeading(), launchEdge2.getHeading())
                .build();
        Grab3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                launchEdge,
                                new Pose(60.561, 29.383),
                                new Pose(39.028, 35.486),
                                grab3
                        )
                )
                .addParametricCallback(0.6 , () ->
                        follower.setMaxPower(0.5)
                )
                .addParametricCallback(1, () ->
                        follower.setMaxPower(1)
                )
                .setLinearHeadingInterpolation(launchEdge.getHeading(), grab3.getHeading())
                .build();
        Launch3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(grab3, launchEdge2)
                )
                .addParametricCallback(0.7, () ->
                        follower.setMaxPower(0.5)
                )
                .addParametricCallback(1, () ->
                        follower.setMaxPower(1)
                )
                .setLinearHeadingInterpolation(grab3.getHeading(), launchEdge2.getHeading())
                .build();
        Exit = follower.pathBuilder()
                .addPath(
                        new BezierLine(launchFinal, exit)
                )
                .build();
    }

    public void initialize() {
        super.reset();
        CommandScheduler.getInstance().setBulkReading(hardwareMap, LynxModule.BulkCachingMode.MANUAL);

        loopTime = new ElapsedTime();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        limelight = new LimelightSubsystem(hardwareMap, LimelightSubsystem.BLUE_APRILTAG_PIPELINE);
        robotStorage = new RobotStorage();

        launcher = new LauncherSubsystem(hardwareMap);
        palete = new PaleteSubsytem(hardwareMap);
        onofrei = new OnofreiSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        intakeKicker = new IntakeKickerSubsystem(hardwareMap);
        rampa = new RampaSubsystem(hardwareMap);
        tilt = new TiltSubsystem(hardwareMap);
        senzorTavan = new SenzorTavanSubsystem(hardwareMap);
        senzorRoata = new SenzorRoataSubsystem(hardwareMap);
        senzorGaura = new SenzorGauraSubsystem(hardwareMap);

        init = new Init(palete, onofrei, rampa, intakeKicker, tilt);

        robotStorage.setSector(0, 2);
        robotStorage.setSector(1, 2);
        robotStorage.setSector(2, 1);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(start);
        buildPaths();
        SequentialCommandGroup autonomousSequence = new SequentialCommandGroup(
                init,
                new ParallelCommandGroup(
                    new FollowPathCommand(follower, preload, true),
                    new InstantCommand(() -> launcher.stop()),
                    new SequentialCommandGroup(
                            new WaitCommand(1000),
                            new LaunchFanFire(robotStorage, telemetryM, palete, onofrei, launcher, rampa, () -> 1300, ALLIANCE)
                    )
                ),

                new SequentialCommandGroup(
                        new TurnToCommand(follower, Math.toRadians(150)),
                        new WaitCommand(200),
                        new ReadMotif(robotStorage, telemetryM, limelight),
                        new TurnToCommand(follower, Math.toRadians(180))
                ),

                new ParallelCommandGroup(
                        new Intake(robotStorage, telemetryM, intake, palete, senzorTavan, senzorRoata, senzorGaura, intakeKicker).withTimeout(6500),
                        new SequentialCommandGroup(
                                new FollowPathCommand(follower, Grab2, true),
                                new FollowPathCommand(follower, Launch2, true)
                        )
                ),
                new WaitCommand(300),
                new LaunchFanFire(robotStorage, telemetryM, palete, onofrei, launcher, rampa, () -> 1330, ALLIANCE),

                new ParallelCommandGroup(
                        new Intake(robotStorage, telemetryM, intake, palete, senzorTavan, senzorRoata, senzorGaura, intakeKicker).withTimeout(6500),
                        new SequentialCommandGroup(
                                new FollowPathCommand(follower, GrabGate, true),
                                new InstantCommand(() -> {
                                    follower.setMaxPower(1);
                                }),
                                new FollowPathCommand(follower, LaunchGate, true)
                        )
                ),
                new WaitCommand(300),
                new LaunchFanFire(robotStorage, telemetryM, follower, palete, onofrei, launcher, rampa, ALLIANCE),

                new ParallelCommandGroup(
                        new Intake(robotStorage, telemetryM, intake, palete, senzorTavan, senzorRoata, senzorGaura, intakeKicker).withTimeout(6500),
                        new SequentialCommandGroup(
                                new FollowPathCommand(follower, GrabGate, true),
                                new InstantCommand(() -> {
                                    follower.setMaxPower(1);
                                }),
                                new FollowPathCommand(follower, LaunchGate, true)
                        )
                ),
                new WaitCommand(300),
                new LaunchFanFire(robotStorage, telemetryM, follower, palete, onofrei, launcher, rampa, ALLIANCE),

                new ParallelCommandGroup(
                        new Intake(robotStorage, telemetryM, intake, palete, senzorTavan, senzorRoata, senzorGaura, intakeKicker),
                        new SequentialCommandGroup(
                                new FollowPathCommand(follower, Grab1, true),
                                new FollowPathCommand(follower, Launch1, true)
                        )
                ),
                new WaitCommand(300),
                new LaunchFanFire(robotStorage, telemetryM, follower, palete, onofrei, launcher, rampa, ALLIANCE),

                new InstantCommand(
                        () -> {
                            launcher.brake();
                        }
                )
        );
        schedule(autonomousSequence);
    }

    @Override
    public void run() {
        super.run();
        follower.update();
        robotStorage.updateAutoEndPose(follower.getPose());

        telemetryM.addData("Loop Time", loopTime.milliseconds());
        telemetryM.addData("X", follower.getPose().getX());
        telemetryM.addData("Y", follower.getPose().getY());
        telemetryM.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetryM.addData("motif", robotStorage.getMotif()[0]);
        telemetryM.update(telemetry);

        loopTime.reset();
    }

    @Override
    public void end() {
        robotStorage.updateAutoEndPose(follower.getPose());
        super.end();
    }
}