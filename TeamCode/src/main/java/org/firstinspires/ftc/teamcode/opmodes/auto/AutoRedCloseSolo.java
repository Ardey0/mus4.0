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
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.commands.Init;
import org.firstinspires.ftc.teamcode.commands.Intake;
import org.firstinspires.ftc.teamcode.commands.LaunchAll;
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
    private final int ALLIANCE = 1; // BLUE

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
    private final Pose start = new Pose(123.6, 121.1, Math.toRadians(129.2));
    private final Pose launchPre = new Pose(88, 87.3, Math.toRadians(135));
    private final Pose motif = new Pose(45, 95, Math.toRadians(-180));
    private final Pose launch1 = new Pose(53, 90, Math.toRadians(-129));
    private final Pose launchEdge = new Pose(89, 80, Math.toRadians(139));
    private final Pose launchFinal = new Pose(86, 99, Math.toRadians(128));
    private final Pose grab1 = new Pose(120, 84.5, Math.toRadians(0));
    private final Pose clearGateSafe = new Pose(13, 73, Math.toRadians(90));
    private final Pose clearGateRisky = new Pose(13.7,64.5, Math.toRadians(180));
    private final Pose grab2 = new Pose(126, 59, Math.toRadians(0));
    private final Pose grab3 = new Pose(17, 35, Math.toRadians(180));
    private final Pose grabGate = new Pose(129, 59, Math.toRadians(31.5));
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
                                launchEdge,
                                new Pose(72.4, 72.4),
                                grab1)
                )
                .setLinearHeadingInterpolation(launchEdge.getHeading(),grab1.getHeading())
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
                .addParametricCallback(0.6, () ->
                        follower.setMaxPower(0.5)
                )
                .addParametricCallback(1, () ->
                        follower.setMaxPower(1)
                )
                .setLinearHeadingInterpolation(grab1.getHeading(), launchFinal.getHeading())
                .build();
        Grab2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                launchPre,
                                new Pose(77.5, 44.2),
                                grab2
                        )
                )
                .addParametricCallback(0.75, () ->
                        follower.setMaxPower(0.5)
                )
                .addParametricCallback(0.99, () ->
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
                .addParametricCallback(0.6, () ->
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
                                new Pose(108.2,56.7),
                                grabGate
                        )
                )
                .addParametricCallback(0.8, () ->
                        follower.setMaxPower(0.7)
                )
                .addParametricCallback(0.99, () ->
                        follower.setMaxPower(1)
                )
                .setLinearHeadingInterpolation(launchEdge.getHeading(),grabGate.getHeading())
                .build();
        LaunchGate = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                grabGate,
                                new Pose(105.8,66.9),
                                launchEdge
                        )
                )
                .addParametricCallback(0.6, () ->
                        follower.setMaxPower(0.5)
                )
                .addParametricCallback(1, () ->
                        follower.setMaxPower(1)
                )
                .setLinearHeadingInterpolation(grabGate.getHeading(), launchEdge.getHeading())
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
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new LaunchAll(robotStorage, telemetryM, follower, palete, onofrei, launcher, rampa, ALLIANCE)
                        )
                ),

                new ParallelCommandGroup(
                        new Intake(robotStorage, telemetryM, intake, palete, senzorTavan, senzorRoata, senzorGaura, intakeKicker).withTimeout(5500),
                        new SequentialCommandGroup(
                                new FollowPathCommand(follower, Grab2, true),
                                new FollowPathCommand(follower, Launch2, true)
                        )
                ),
                new LaunchAll(robotStorage, telemetryM, follower, palete, onofrei, launcher, rampa, ALLIANCE),

                new ParallelCommandGroup(
                        new Intake(robotStorage, telemetryM, intake, palete, senzorTavan, senzorRoata, senzorGaura, intakeKicker).withTimeout(5500),
                        new SequentialCommandGroup(
                                new FollowPathCommand(follower, GrabGate, true),
                                new InstantCommand(() -> {
                                    follower.setMaxPower(1);
                                }),
                                new WaitCommand(600),
                                new FollowPathCommand(follower, LaunchGate, true)
                        )
                ),
                new LaunchAll(robotStorage, telemetryM, follower, palete, onofrei, launcher, rampa, ALLIANCE),

                new ParallelCommandGroup(
                        new Intake(robotStorage, telemetryM, intake, palete, senzorTavan, senzorRoata, senzorGaura, intakeKicker).withTimeout(5500),
                        new SequentialCommandGroup(
                                new FollowPathCommand(follower, GrabGate, true),
                                new InstantCommand(() -> {
                                    follower.setMaxPower(1);
                                }),
                                new WaitCommand(1000),
                                new FollowPathCommand(follower, LaunchGate, true)
                        )
                ),
                new LaunchAll(robotStorage, telemetryM, follower, palete, onofrei, launcher, rampa, ALLIANCE),


                new ParallelCommandGroup(
                        new Intake(robotStorage, telemetryM, intake, palete, senzorTavan, senzorRoata, senzorGaura, intakeKicker).withTimeout(4000),
                        new SequentialCommandGroup(
                                new FollowPathCommand(follower, Grab1, true),
                                new FollowPathCommand(follower, Launch1, true)
                        )
                ),
                new LaunchAll(robotStorage, telemetryM, follower, palete, onofrei, launcher, rampa, ALLIANCE),

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




