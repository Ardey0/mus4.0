package org.firstinspires.ftc.teamcode.opmodes.auto;


import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
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
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.commands.Init;
import org.firstinspires.ftc.teamcode.commands.IntakeBallIndexing;
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
import org.firstinspires.ftc.teamcode.subsystems.SenzorGauraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SenzorRoataSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SenzorTavanSubsystem;

@Autonomous
public class AutoRedFarHuman extends CommandOpMode {
    private final int ALLIANCE = 1; // RED
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
    public PathChain Grab1;
    public PathChain Pickup1;
    public PathChain LaunchHuman;
    public PathChain Grab2;
    public PathChain Pickup2;
    public PathChain LaunchMiddle;
    public PathChain GrabGate;
    public PathChain LaunchGate;
    public PathChain GrabHuman;
    public PathChain PickupHuman;
    public PathChain GrabHuman2;
    public PathChain PickupHuman2;
    public PathChain HumanToLaunch;
    private PathChain Exit;
    public PathChain LaunchHuman2;


    private final Pose start = new Pose(88, 8.740, Math.toRadians(180));
    private final Pose launchHuman = new Pose(87.5, 20, Math.toRadians(159));
    private final Pose grab1 = new Pose(105, 36, Math.toRadians(0));
    private final Pose pickup1 = new Pose(132, 36, Math.toRadians(0));
    private final Pose grab2 = new Pose(105, 58, Math.toRadians(0));
    private final Pose pickup2 = new Pose(133, 58, Math.toRadians(0));
    private final Pose launchMiddle = new Pose(93.7, 87.40, Math.toRadians(150));
    private final Pose grabHuman = new Pose(135, 27, Math.toRadians(-90));
    private final Pose pickupHuman = new Pose(135, 10, Math.toRadians(-90));
    private final Pose grabHuman2 = new Pose(135, 27, Math.toRadians(90));
    private final Pose pickupHuman2 = new Pose(135, 49, Math.toRadians(90));
    private final Pose exit = new Pose(105, 15, Math.toRadians(135));

    public void buildPaths() {
        preload = follower.pathBuilder()
                .addPath(
                        new BezierLine(start, launchHuman)
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), launchHuman.getHeading())
                .build();

        Grab1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(launchHuman, grab1)
                )
                .setConstantHeadingInterpolation(grab1.getHeading())
                .build();

        Pickup1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(grab1, pickup1)
                )
                .setLinearHeadingInterpolation(grab1.getHeading(), pickup1.getHeading())
                .build();

        LaunchHuman = follower.pathBuilder()
                .addPath(
                        new BezierLine(pickupHuman, launchHuman)
                )
                .setLinearHeadingInterpolation(pickupHuman.getHeading(), launchHuman.getHeading())
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
                        new BezierLine(pickup2, launchMiddle)
                )
                .setLinearHeadingInterpolation(pickup2.getHeading(), launchMiddle.getHeading())
                .build();

        LaunchHuman2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(pickupHuman2, launchHuman)
                )
                .setLinearHeadingInterpolation(pickupHuman2.getHeading(), launchHuman.getHeading())
                .build();

        GrabHuman = follower.pathBuilder()
                .addPath(
                        new BezierLine(launchHuman, grabHuman)
                )
                .setLinearHeadingInterpolation(launchHuman.getHeading(), grabHuman.getHeading())
                .build();
        PickupHuman = follower.pathBuilder()
                .addPath(
                        new BezierLine(grabHuman, pickupHuman)
                )
                .setLinearHeadingInterpolation(grabHuman.getHeading(), pickupHuman.getHeading())
                .build();

        GrabHuman2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(launchHuman, grabHuman2)
                )
                .setLinearHeadingInterpolation(launchHuman.getHeading(), grabHuman2.getHeading())
                .build();
        PickupHuman2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(grabHuman2, pickupHuman2)
                )
                .setLinearHeadingInterpolation(grabHuman2.getHeading(), pickupHuman2.getHeading())
                .build();

        Exit = follower.pathBuilder()
                .addPath(
                        new BezierLine(launchHuman, exit)
                )
                .setLinearHeadingInterpolation(launchHuman.getHeading(), exit.getHeading())
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
                                    return verzi == 1 && mov == 2 && robotStorage.getMotif()[0] != 0;
                                }
                        ),
                        new FollowPathCommand(follower, preload, true, 1)
                ),
                new FollowPathCommand(follower, Grab1, true, 1),
                new ParallelCommandGroup(
                        new IntakeBallIndexing(robotStorage, telemetryM, intake, palete, senzorTavan, senzorRoata, senzorGaura, intakeKicker).withTimeout(4000),
                        new SequentialCommandGroup(
                                new FollowPathCommand(follower, Pickup1, true, 0.2),
                                new FollowPathCommand(follower, LaunchHuman, true, 0.7)
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
                new ParallelCommandGroup(
                        new IntakeBallIndexing(robotStorage, telemetryM, intake, palete, senzorTavan, senzorRoata, senzorGaura, intakeKicker).withTimeout(6000),
                        new SequentialCommandGroup(
                                new FollowPathCommand(follower, GrabHuman, true, 1),
                                new FollowPathCommand(follower, PickupHuman, true, 0.2),
                                new FollowPathCommand(follower, LaunchHuman, true, 0.7)
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
                new ParallelCommandGroup(
                        new IntakeBallIndexing(robotStorage, telemetryM, intake, palete, senzorTavan, senzorRoata, senzorGaura, intakeKicker).withTimeout(6000),
                        new SequentialCommandGroup(
                                new FollowPathCommand(follower, GrabHuman2, true, 1),
                                new FollowPathCommand(follower, PickupHuman2, true, 0.2),
                                new WaitCommand(500),
                                new FollowPathCommand(follower, LaunchHuman2, true, 0.7)
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
                new FollowPathCommand(follower, Exit, true),
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






