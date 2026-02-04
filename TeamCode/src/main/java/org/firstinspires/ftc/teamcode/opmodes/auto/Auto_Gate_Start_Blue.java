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
import org.firstinspires.ftc.teamcode.commands.IntakeBall;
import org.firstinspires.ftc.teamcode.commands.LaunchAllBalls;
import org.firstinspires.ftc.teamcode.commands.LaunchMotifBalls;
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
import org.firstinspires.ftc.teamcode.subsystems.SenzorRoataSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SenzorTavanSubsystem;

@Autonomous
public class Auto_Gate_Start_Blue extends CommandOpMode {
    private final int ALLIANCE = 0; // BLUE

    private TelemetryManager telemetryM;

    private LauncherSubsystem launcher;
    private PaleteSubsytem palete;
    private OnofreiSubsystem onofrei;
    private IntakeSubsystem intake;
    private IntakeKickerSubsystem intakeKicker;
    private RampaSubsystem rampa;
    private SenzorTavanSubsystem senzorTavan;
    private SenzorRoataSubsystem senzorRoata;
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
    private final Pose start = new Pose(19, 120, Math.toRadians(-129));
    private final Pose launchPre = new Pose(45, 95, Math.toRadians(-129));
    private final Pose motif = new Pose(45, 95, Math.toRadians(-180));
    private final Pose launch1 = new Pose(53, 90, Math.toRadians(-129));
    private final Pose launch2 = new Pose(52, 90, Math.toRadians(-130));
    private final Pose launch3 = new Pose(52, 90, Math.toRadians(-130));
    private final Pose grab1 = new Pose(22.5, 82, Math.toRadians(180));
    private final Pose clearGate = new Pose(15, 73, Math.toRadians(90));
    private final Pose grab2 = new Pose(14, 58, Math.toRadians(180));
    private final Pose grab3 = new Pose(14, 35, Math.toRadians(180));
    private final Pose exit = new Pose(40, 80, Math.toRadians(-135));

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
                        new BezierLine(launchPre, grab1)
                )
                .setLinearHeadingInterpolation(180, grab1.getHeading())
                .build();


        ClearGate = follower.pathBuilder()
                .addPath(
                        new BezierLine(grab1, clearGate)
                )
                .setConstantHeadingInterpolation(clearGate.getHeading())
                .build();
        Launch1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(clearGate, launch1)
                )
                .setLinearHeadingInterpolation(clearGate.getHeading(), launch1.getHeading())
                .build();
        Grab2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                launch1,
                                new Pose(47.981, 53.975),
                                new Pose(51.262, 61.897),
                                grab2
                        )
                )
                .setLinearHeadingInterpolation(launch1.getHeading(), grab2.getHeading())
                .build();
        Launch2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                grab2,
                                new Pose(44.168, 54.873),
                                launch2)
                )
                .setLinearHeadingInterpolation(grab2.getHeading(), launch2.getHeading())
                .build();
        Grab3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                launch2,
                                new Pose(60.561, 29.383),
                                new Pose(39.028, 35.486),
                                grab3
                        )
                )
                .setLinearHeadingInterpolation(launch2.getHeading(), grab3.getHeading())
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
//
//        Pickup1Wiggle = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(pickup1, pickup1Wiggle)
//                )
//                .addPath(
//                        new BezierLine(pickup1Wiggle, pickup1)
//                )
//                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .build();
//
//        LaunchHuman2 = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(pickup2, launchHuman)
//                )
//                .setLinearHeadingInterpolation(pickup2.getHeading(), launchHuman.getHeading())
//                .build();
//
//        Grab2 = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(launchHuman, grab2)
//                )
//                .setLinearHeadingInterpolation(launchHuman.getHeading(), grab2.getHeading())
//                .build();
//
//        Pickup2 = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(grab2, pickup2)
//                )
//                .setLinearHeadingInterpolation(grab2.getHeading(), pickup2.getHeading())
//                .build();
//
//        LaunchMiddle = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(grab2, launchMiddle)
//                )
//                .setLinearHeadingInterpolation(grab2.getHeading(), Math.toRadians(-143))
//                .build();
//
//        GrabGate = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(launchMiddle, grabGate)
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(-150), Math.toRadians(180))
//                .build();
//
//        LaunchGate = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(grabGate, launchGate)
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-135))
//                .build();
//
//        GrabHuman = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(launchHuman, grabHuman)
//                )
//                .setLinearHeadingInterpolation(launchHuman2.getHeading(), Math.toRadians(180))
//                .build();
//
//        PickupHuman = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(grabHuman, pickupHuman)
//                )
//                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .build();
//
//        HumanToLaunch = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(pickupHuman, launchHuman2)
//                )
//                .setLinearHeadingInterpolation(pickupHuman.getHeading(), launchHuman2.getHeading())
//                .build();
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
        senzorTavan = new SenzorTavanSubsystem(hardwareMap);
        senzorRoata = new SenzorRoataSubsystem(hardwareMap);

        init = new Init(palete, onofrei, rampa, intakeKicker);

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
                                new LaunchAllBalls(robotStorage, telemetryM, follower, palete, onofrei, launcher, rampa, ALLIANCE)
                        )
                ),
                new SequentialCommandGroup(
                        new TurnToCommand(follower, Math.toRadians(150)),
                        new WaitCommand(500),
                        new ReadMotif(robotStorage, telemetryM, limelight),
                        new TurnToCommand(follower, Math.toRadians(180))
                ),
                new ParallelCommandGroup(
                        new IntakeBall(robotStorage, telemetryM, intake, palete, senzorTavan, senzorRoata, intakeKicker).withTimeout(6500),
                        new SequentialCommandGroup(
                                new FollowPathCommand(follower, Grab1, true),
                                new FollowPathCommand(follower, ClearGate, true),
                                new WaitCommand(300),
                                new FollowPathCommand(follower, Launch1, true)
                        )
                ),
                new ConditionalCommand(
                        new LaunchMotifBalls(robotStorage, telemetryM, follower, palete, onofrei, launcher, rampa, ALLIANCE),
                        new LaunchAllBalls(robotStorage, telemetryM, follower, palete, onofrei, launcher, rampa, ALLIANCE),
                        () -> {
                            int verzi = 0, mov = 0;
                            for (int i = 0; i <= 2; i++) {
                                if (robotStorage.getSectorColor(i) == 1) verzi++;
                                if (robotStorage.getSectorColor(i) == 2) mov++;
                            }
                            return verzi == 1 && mov == 2 && robotStorage.getMotif()[0] != 0;
                        }
                ),
                new ParallelCommandGroup(
                        new IntakeBall(robotStorage, telemetryM, intake, palete, senzorTavan, senzorRoata, intakeKicker).withTimeout(6500),
                        new SequentialCommandGroup(
                                new FollowPathCommand(follower, Grab2, true),
                                new FollowPathCommand(follower, Launch2, true)
                        )
                ),
                new ConditionalCommand(
                        new LaunchMotifBalls(robotStorage, telemetryM, follower, palete, onofrei, launcher, rampa, ALLIANCE),
                        new LaunchAllBalls(robotStorage, telemetryM, follower, palete, onofrei, launcher, rampa, ALLIANCE),
                        () -> {
                            int verzi = 0, mov = 0;
                            for (int i = 0; i <= 2; i++) {
                                if (robotStorage.getSectorColor(i) == 1) verzi++;
                                if (robotStorage.getSectorColor(i) == 2) mov++;
                            }
                            return verzi == 1 && mov == 2 && robotStorage.getMotif()[0] != 0;
                        }
                ),
                new ParallelCommandGroup(
                        new IntakeBall(robotStorage, telemetryM, intake, palete, senzorTavan, senzorRoata, intakeKicker).withTimeout(6000),
                        new SequentialCommandGroup(
                                new FollowPathCommand(follower, Grab3, true),
                                new FollowPathCommand(follower, Launch3, true)
                        )
                ),
                new ParallelCommandGroup(
                        new ConditionalCommand(
                                new LaunchMotifBalls(robotStorage, telemetryM, follower, palete, onofrei, launcher, rampa, ALLIANCE),
                                new LaunchAllBalls(robotStorage, telemetryM, follower, palete, onofrei, launcher, rampa, ALLIANCE),
                                () -> {
                                    int verzi = 0, mov = 0;
                                    for (int i = 0; i <= 2; i++) {
                                        if (robotStorage.getSectorColor(i) == 1) verzi++;
                                        if (robotStorage.getSectorColor(i) == 2) mov++;
                                    }
                                    return verzi == 1 && mov == 2 && robotStorage.getMotif()[0] != 0;
                                }
                        ),
                        new SpitBalls(intake).withTimeout(1000)
                ),
                new InstantCommand(
                        () -> {
                            launcher.brake();
                        }
                ),
                new FollowPathCommand(follower, Exit, true)
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
}