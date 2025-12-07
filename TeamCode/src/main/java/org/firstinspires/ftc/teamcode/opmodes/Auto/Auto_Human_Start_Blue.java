package org.firstinspires.ftc.teamcode.opmodes.Auto;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.commands.Init;
import org.firstinspires.ftc.teamcode.commands.IntakeBall;
import org.firstinspires.ftc.teamcode.commands.LaunchMotifBalls;
import org.firstinspires.ftc.teamcode.commands.ReadMotif;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensorSubsystem;
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
    private LaunchMotifBalls launchMotifBalls;
    private ReadMotif readMotif;

    private int pathState;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private double time;
    private Follower follower;

    public PathChain preload;
    public PathChain GrabHuman;
    public PathChain LaunchHuman;
    public PathChain GrabMiddle;
    public PathChain LaunchMiddle;
    public PathChain GrabGate;
    public PathChain LaunchGate;
    private final Pose start = new Pose(55.700, 8.740,Math.toRadians(180));
    private final Pose launchHuman = new Pose(58.540, 23.500);
    private final Pose grabHuman = new Pose(41.96, 37.750);
    private final Pose grabMiddle = new Pose(41.96, 62.5);
    private final Pose launchMiddle = new Pose(55.700, 8.740);
    private final Pose grabGate = new Pose(41.96, 87);
    private final Pose launchGate = new Pose(45, 87);

    public void buildPaths() {
        preload = follower.pathBuilder()
                .addPath(
                        new BezierLine(start, launchHuman)
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-153))
                .build();

        GrabHuman = follower.pathBuilder()
                .addPath(
                        new BezierLine(launchHuman, grabHuman)
                )
                .setLinearHeadingInterpolation(Math.toRadians(-160), Math.toRadians(180))
                .build();

        LaunchHuman = follower.pathBuilder()
                .addPath(
                        new BezierLine(grabHuman, launchHuman)
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-160))
                .build();

        GrabMiddle = follower.pathBuilder()
                .addPath(
                        new BezierLine(launchHuman, grabMiddle)
                )
                .setLinearHeadingInterpolation(Math.toRadians(-160), Math.toRadians(180))
                .build();

        LaunchMiddle = follower.pathBuilder()
                .addPath(
                        new BezierLine(grabMiddle, launchMiddle)
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-135))
                .build();

        GrabGate = follower.pathBuilder()
                .addPath(
                        new BezierLine(launchMiddle, grabGate)
                )
                .setLinearHeadingInterpolation(Math.toRadians(-135), Math.toRadians(180))
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
        launchMotifBalls = new LaunchMotifBalls(robotStorage, telemetry, palete, onofrei, launcher, () -> true);

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
                new FollowPathCommand(follower, preload, true),
                launchMotifBalls
        );
        schedule(autonomousSequence);
    }

    @Override
    public void run() {
        super.run();
        follower.update();
//        telemetryData.addData("X", follower.getPose().getX());
//        telemetryData.addData("Y", follower.getPose().getY());
//        telemetryData.addData("Heading", follower.getPose().getHeading());
//        telemetryData.update();
    }
}
