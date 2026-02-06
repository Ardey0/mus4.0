package org.firstinspires.ftc.teamcode.opmodes.auto;


import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
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
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.commands.Init;
import org.firstinspires.ftc.teamcode.commands.IntakeBallIndexing;
import org.firstinspires.ftc.teamcode.commands.LaunchAllBalls;
import org.firstinspires.ftc.teamcode.commands.LaunchMotifBalls;
import org.firstinspires.ftc.teamcode.commands.ReadMotif;
import org.firstinspires.ftc.teamcode.commands.SpitBalls;
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

@Autonomous
public class AutoBlueFar extends CommandOpMode {
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
    private SenzorGauraSubsystem senzorGaura;
    private RobotStorage robotStorage;
    private LimelightSubsystem limelight;

    private Init init;
    private ReadMotif readMotif;

    private ElapsedTime loopTime;

    private Follower follower;


    public PathChain preload;
    private PathChain Exit;
    public PathChain Grab1;
    public PathChain Pickup1;
    public PathChain LaunchHuman;
    public PathChain LaunchHuman2;
    public PathChain Grab2;
    public PathChain Pickup2;
    public PathChain LaunchMiddle;
    public PathChain GrabGate;
    public PathChain LaunchGate;
    public PathChain PickupHuman;

    private final Pose start = new Pose(55.700, 8.740, Math.toRadians(180));
    private final Pose exit = new Pose(43, 13, Math.toRadians(-135));
    private final Pose launchHuman = new Pose(53, 18, Math.toRadians(-155));
    private final Pose grab1 = new Pose(39, 36, Math.toRadians(180));
    private final Pose pickup1 = new Pose(13, 36, Math.toRadians(180));
    private final Pose grab2 = new Pose(40, 58.5, Math.toRadians(180));
    private final Pose pickup2 = new Pose(12, 58.5, Math.toRadians(180));
    private final Pose launchMiddle = new Pose(50.700, 87.40, Math.toRadians(-153));
    private final Pose grabHuman = new Pose(32, 11.5, Math.toRadians(180));
    private final Pose pickupHuman = new Pose(9, 11.5, Math.toRadians(180));
    private final Pose grabGate = new Pose(38, 87);
    private final Pose launchGate = new Pose(45, 87);

    public void buildPaths() {
        preload = follower.pathBuilder()
                .addPath(
                        new BezierLine(start, launchHuman)
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), launchHuman.getHeading())
                .build();

        Grab1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(launchHuman,
                                new Pose(47.958, 45.697),
                                new Pose(27.064, 33.978),
                                pickup1)
                )
                .setLinearHeadingInterpolation(launchHuman.getHeading(), grab1.getHeading())
                .build();

        Exit = follower.pathBuilder()
                .addPath(
                        new BezierLine(launchHuman, exit)
                )
                .setLinearHeadingInterpolation(launchHuman.getHeading(), exit.getHeading())
                .build();

        Pickup1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(grab1, pickup1)
                )
                .setConstantHeadingInterpolation(grab1.getHeading() )
                .build();

        LaunchHuman = follower.pathBuilder()
                .addPath(
                        new BezierLine(pickup1, launchHuman)
                )
                .setLinearHeadingInterpolation(pickup1.getHeading(), launchHuman.getHeading())
                .build();

        LaunchHuman2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(pickup2, launchHuman)
                )
                .setLinearHeadingInterpolation(pickup2.getHeading(), launchHuman.getHeading())
                .build();

        Grab2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(launchHuman, grab2)
                )
                .setLinearHeadingInterpolation(launchHuman.getHeading(), grab2.getHeading())
                .build();

        Pickup2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(grab2, pickup2)
                )
                .setLinearHeadingInterpolation(grab2.getHeading(), pickup2.getHeading())
                .build();

        LaunchMiddle = follower.pathBuilder()
                .addPath(
                        new BezierLine(grab2, launchMiddle)
                )
                .setLinearHeadingInterpolation(grab2.getHeading(), Math.toRadians(-143))
                .build();

        GrabGate = follower.pathBuilder()
                .addPath(
                        new BezierLine(launchMiddle, grabGate)
                )
                .setLinearHeadingInterpolation(Math.toRadians(-150), Math.toRadians(180))
                .build();

        LaunchGate = follower.pathBuilder()
                .addPath(
                        new BezierLine(grabGate, launchGate)
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-135))
                .build();

//        GrabHuman = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(launchHuman, grabHuman)
//                )
//                .setLinearHeadingInterpolation(launchHuman2.getHeading(), Math.toRadians(180))
//                .build();

        PickupHuman = follower.pathBuilder()
                .addPath(
                        new BezierLine(grabHuman, pickupHuman)
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

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
        senzorGaura = new SenzorGauraSubsystem(hardwareMap);

        init = new Init(palete, onofrei, rampa, intakeKicker);

        readMotif = new ReadMotif(robotStorage, telemetryM, limelight);

        robotStorage.setSector(0, 2);
        robotStorage.setSector(1, 2);
        robotStorage.setSector(2, 1);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(start);
        buildPaths();

        SequentialCommandGroup autonomousSequence = new SequentialCommandGroup(
                init,
                readMotif,
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
                                    return verzi == 1 && mov == 2;
                                }
                        ),
                        new FollowPathCommand(follower, preload, true, 1)
                ),
                new ParallelCommandGroup(
                        new IntakeBallIndexing(robotStorage, telemetryM, intake, palete, senzorTavan, senzorRoata, senzorGaura, intakeKicker).withTimeout(6700),
                        new SequentialCommandGroup(
                                new FollowPathCommand(follower, Grab1, true),
                                new FollowPathCommand(follower, LaunchHuman, true, 0.9)
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
                                    return verzi == 1 && mov == 2;
                                }
                        ),
                        new SpitBalls(intake).withTimeout(1000)
                ),
                new FollowPathCommand(follower, Grab2, true, 1),
                new ParallelCommandGroup(
                        new IntakeBallIndexing(robotStorage, telemetryM, intake, palete, senzorTavan, senzorRoata, senzorGaura, intakeKicker).withTimeout(6700),
                        new SequentialCommandGroup(
                                new FollowPathCommand(follower, Pickup2, true, 0.2),
                                new FollowPathCommand(follower, LaunchHuman2, true, 0.9)
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
                                    return verzi == 1 && mov == 2;
                                }
                        ),
                        new SpitBalls(intake).withTimeout(1000)
                ),
                new FollowPathCommand(follower, Exit, true),
                new InstantCommand(
                        () -> {
                            launcher.brake();
                        }
                ));
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
        telemetryM.addData("motif", robotStorage.getMotif()[0]);
        telemetryM.update(telemetry);

        loopTime.reset();
    }
}






