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
import org.firstinspires.ftc.teamcode.commands.IntakeIndexing;
import org.firstinspires.ftc.teamcode.commands.Intake;
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
public class AutoRedCloseSolo extends CommandOpMode {
    private final int ALLIANCE = 1; // RED

    private TelemetryManager telemetryM;

    private LauncherSubsystem launcher;
    private PaleteSubsytem palete;
    private OnofreiSubsystem onofrei;
    private IntakeSubsystem intake;
    private IntakeKickerSubsystem intakeKicker;
    private RampaSubsystem rampa;
    private TiltSubsystem tilt;
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
    public PathChain GrabGate;
    public PathChain LaunchGate;

    private final Pose start = new Pose(123, 120, Math.toRadians(129.3));
    private final Pose launchPre = new Pose(97, 95, Math.toRadians(135));
    private final Pose motif = new Pose(45, 95, Math.toRadians(-180));
    private final Pose launch = new Pose(90, 90, Math.toRadians(134));
    private final Pose launchFinal = new Pose(87, 102, Math.toRadians(131));
    private final Pose launch3 = new Pose(86, 100, Math.toRadians(131));
    private final Pose grab1 = new Pose(121, 81, Math.toRadians(0));
    private final Pose clearGate = new Pose(128, 73 , Math.toRadians(90));
    private final Pose grab2 = new Pose(126, 55, Math.toRadians(0));
    private final Pose grab3 = new Pose(130, 31, Math.toRadians(0));
    private final Pose exit = new Pose(150, 90, Math.toRadians(-135));
    private final Pose grabGate = new Pose(132, 57, Math.toRadians(34.5));



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
                        new BezierLine(launch, grab1)
                )
                .addParametricCallback(0.6, () ->
                        follower.setMaxPower(0.5)
                )
                .addParametricCallback(1, () ->
                        follower.setMaxPower(1)
                )
                .setConstantHeadingInterpolation(grab1.getHeading())
                .build();

        GrabGate = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                launch,
                                new Pose(99.1, 47.5),
                                grabGate)
                )
                .addParametricCallback(0.8, () ->
                        follower.setMaxPower(0.5)
                )
                .addParametricCallback(1, () ->
                        follower.setMaxPower(1)
                )
                .setLinearHeadingInterpolation(launch.getHeading(), grabGate.getHeading())
                .build();
        LaunchGate = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                grabGate,
                                new Pose(96.6, 61.8),
                                launch)
                )
                .setLinearHeadingInterpolation(grab2.getHeading(), launch.getHeading())
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
                                new Pose(83.25, 63.62),
                                new Pose(75.04, 51.58),
                                grab2
                        )
                )
                .addParametricCallback(0.6, () ->
                        follower.setMaxPower(0.5)
                )
                .addParametricCallback(1, () ->
                        follower.setMaxPower(1)
                )
                .setTangentHeadingInterpolation()
                .build();

        Launch2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                grab2,
                                new Pose(97.832, 54.873),
                                launch)
                )
                .setLinearHeadingInterpolation(grab2.getHeading(), launch.getHeading())
                .build();

        Grab3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                launch,
                                new Pose(78, 61),
                                new Pose(60, 29),
                                grab3
                        )
                )
                .setLinearHeadingInterpolation(launch.getHeading(), grab3.getHeading())
                .build();
        Launch3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(grab3, launch3)
                )
                .setLinearHeadingInterpolation(grab3.getHeading(), launch3.getHeading())
                .build();
        Exit = follower.pathBuilder()
                .addPath(
                        new BezierLine(launch3, exit)
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
        tilt = new TiltSubsystem(hardwareMap);
        rampa = new RampaSubsystem(hardwareMap);
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
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new LaunchAll(robotStorage, telemetryM, follower, palete, onofrei, launcher, rampa, ALLIANCE)
                            )
                ),
                new ParallelCommandGroup(
                        new Intake(robotStorage, telemetryM, intake, palete, senzorTavan, senzorRoata, senzorGaura, intakeKicker).withTimeout(6000),
                        new SequentialCommandGroup(
                                new FollowPathCommand(follower, Grab2, true),
                                new FollowPathCommand(follower, Launch2, true)
                        )
                ),
                new ParallelCommandGroup(
                        new LaunchAll(robotStorage, telemetryM, follower, palete, onofrei, launcher, rampa, ALLIANCE),
                        new SpitBalls(intake).withTimeout(1000)
                ),
                new ParallelCommandGroup(
                        new Intake(robotStorage, telemetryM, intake, palete, senzorTavan, senzorRoata, senzorGaura, intakeKicker).withTimeout(6000),
                        new SequentialCommandGroup(
                                new FollowPathCommand(follower, GrabGate, true),
                                new InstantCommand(() -> {
                                    follower.setMaxPower(1);   }
                                ),
                                new WaitCommand(400),
                                new FollowPathCommand(follower, LaunchGate, true)
                        )
                ),
                new ParallelCommandGroup(
                        new LaunchAll(robotStorage, telemetryM, follower, palete, onofrei, launcher, rampa, ALLIANCE),
                        new SpitBalls(intake).withTimeout(1000)
                ),
//                new ParallelCommandGroup(
//                        new Intake(robotStorage, telemetryM, intake, palete, senzorTavan, senzorRoata, senzorGaura, intakeKicker).withTimeout(6000),
//                        new SequentialCommandGroup(
//                                new FollowPathCommand(follower, GrabGate, true),
//                                new InstantCommand(() -> {
//                                    follower.setMaxPower(1);   }
//                                ),
//                                new WaitCommand(300),
//                                new FollowPathCommand(follower, LaunchGate, true)
//                        )
//                ),
//                new ParallelCommandGroup(
//                        new LaunchAll(robotStorage, telemetryM, follower, palete, onofrei, launcher, rampa, ALLIANCE),
//                        new SpitBalls(intake).withTimeout(1000)
//                ),
                new TurnToCommand(follower, Math.toRadians(0)),
                new ParallelCommandGroup(
                        new Intake(robotStorage, telemetryM, intake, palete, senzorTavan, senzorRoata, senzorGaura, intakeKicker).withTimeout(6000),
                        new SequentialCommandGroup(
                                new FollowPathCommand(follower, Grab1, true),
                                new FollowPathCommand(follower, Launch1, true)
                        )
                ),
                new ParallelCommandGroup(
                        new LaunchAll(robotStorage, telemetryM, follower, palete, onofrei, launcher, rampa, ALLIANCE),
                        new SpitBalls(intake).withTimeout(1000)
                ),
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
        telemetryM.addData("Heading", follower.getPose().getHeading());
        telemetryM.update(telemetry);

        loopTime.reset();
    }
}