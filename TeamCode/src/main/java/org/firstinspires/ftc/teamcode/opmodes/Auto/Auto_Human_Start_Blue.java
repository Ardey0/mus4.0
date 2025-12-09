package org.firstinspires.ftc.teamcode.opmodes.Auto;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.commands.Init;
import org.firstinspires.ftc.teamcode.commands.IntakeBall;
import org.firstinspires.ftc.teamcode.commands.LaunchAllBalls;
import org.firstinspires.ftc.teamcode.commands.LaunchMotifBalls;
import org.firstinspires.ftc.teamcode.commands.ReadMotif;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OnofreiSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PaleteSubsytem;
import org.firstinspires.ftc.teamcode.subsystems.RobotStorage;

@Autonomous
public class Auto_Human_Start_Blue extends CommandOpMode {
    TelemetryData telemetryData = new TelemetryData(telemetry);

    private LauncherSubsystem launcher;
    private PaleteSubsytem palete;
    private OnofreiSubsystem onofrei;
    private IntakeSubsystem intake;
    private ColorSensorSubsystem sensor;
    private RobotStorage robotStorage;
    private LimelightSubsystem limelight;

    private Init init;
    private IntakeBall intakeBall;
    private LaunchAllBalls launchAllBallFar;
    private LaunchAllBalls launchAllBallClose;
    private ReadMotif readMotif;

    private int pathState;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private double time;
    private Follower follower;

    public PathChain preload;
    public PathChain GrabHuman;
    public PathChain PickupHuman;
    public PathChain LaunchHuman;
    public PathChain GrabMiddle;
    public PathChain PickupMiddle;
    public PathChain LaunchMiddle;
    public PathChain GrabGate;
    public PathChain LaunchGate;
    private final Pose start = new Pose(55.700, 8.740, Math.toRadians(180));
    private final Pose launchHuman = new Pose(60.540, 23, Math.toRadians(-157));
    private final Pose grabHuman = new Pose(35, 36.5, Math.toRadians(180));
    private final Pose pickupHuman = new Pose(10, 36.5, Math.toRadians(180));
    private final Pose grabMiddle = new Pose(38, 58.5, Math.toRadians(180));
    private final Pose pickupMiddle = new Pose(10, 58.5, Math.toRadians(180));
    private final Pose launchMiddle = new Pose(50.700, 87.40, Math.toRadians(-155));
    private final Pose grabGate = new Pose(38, 87);
    private final Pose launchGate = new Pose(45, 87);

    public void buildPaths() {
        preload = follower.pathBuilder()
                .addPath(
                        new BezierLine(start, launchHuman)
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), launchHuman.getHeading())
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

        LaunchHuman = follower.pathBuilder()
                .addPath(
                        new BezierLine(grabHuman, launchHuman)
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), launchHuman.getHeading())
                .build();

        GrabMiddle = follower.pathBuilder()
                .addPath(
                        new BezierLine(launchHuman, grabMiddle)
                )
                .setLinearHeadingInterpolation(launchHuman.getHeading(), grabMiddle.getHeading())
                .build();
        PickupMiddle = follower.pathBuilder()
                .addPath(
                        new BezierLine(grabMiddle, pickupMiddle)
                )
                .setLinearHeadingInterpolation(grabMiddle.getHeading(), pickupMiddle.getHeading())
                .build();

        LaunchMiddle = follower.pathBuilder()
                .addPath(
                        new BezierLine(grabMiddle, launchMiddle)
                )
                .setLinearHeadingInterpolation(grabMiddle.getHeading(), Math.toRadians(-150))
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
    }

    public void initialize() {
        super.reset();

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        limelight = new LimelightSubsystem(hardwareMap);
        robotStorage = new RobotStorage();

        launcher = new LauncherSubsystem(hardwareMap);
        palete = new PaleteSubsytem(hardwareMap);
        onofrei = new OnofreiSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        sensor = new ColorSensorSubsystem(hardwareMap);

        init = new Init(palete, onofrei);
        intakeBall = new IntakeBall(robotStorage, telemetry, intake, palete, sensor);
        launchAllBallFar = new LaunchAllBalls(robotStorage, telemetry, palete, onofrei, launcher, () -> true);
        launchAllBallClose = new LaunchAllBalls(robotStorage, telemetry, palete, onofrei, launcher, () -> false);

        readMotif = new ReadMotif(robotStorage, telemetry, limelight);


        robotStorage.setSector(0, 2);
        robotStorage.setSector(1, 2);
        robotStorage.setSector(2, 1);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(start);
        buildPaths();

        SequentialCommandGroup autonomousSequence = new SequentialCommandGroup(
                init,
                readMotif,
                new FollowPathCommand(follower, preload),
                new LaunchAllBalls(robotStorage, telemetry, palete, onofrei, launcher, () -> true),
                new FollowPathCommand(follower, GrabHuman),
                new ParallelCommandGroup(
                        new IntakeBall(robotStorage, telemetry, intake, palete, sensor),
                        new FollowPathCommand(follower, PickupHuman, true, 0.2)
                ),
                new FollowPathCommand(follower, LaunchHuman),
                new LaunchAllBalls(robotStorage, telemetry, palete, onofrei, launcher, () -> true),
                new FollowPathCommand(follower, GrabMiddle),
                new ParallelCommandGroup(
                        new IntakeBall(robotStorage, telemetry, intake, palete, sensor),
                        new FollowPathCommand(follower, PickupMiddle, true, 0.2)
                ),
                new FollowPathCommand(follower, LaunchMiddle),
                new LaunchAllBalls(robotStorage, telemetry, palete, onofrei, launcher, () -> true)
//                launchAllBallFar
        );
        schedule(autonomousSequence);
    }

    @Override
    public void run() {
        super.run();
        follower.update();
        telemetryData.addData("X", follower.getPose().getX());
        telemetryData.addData("Y", follower.getPose().getY());
        telemetryData.addData("Heading", follower.getPose().getHeading());
        telemetryData.update();
    }
}
