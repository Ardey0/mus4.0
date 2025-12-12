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
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
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
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OnofreiSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PaleteSubsytem;
import org.firstinspires.ftc.teamcode.subsystems.RobotStorage;

@Autonomous
public class Auto_Human_Start_Blue extends CommandOpMode {
    private TelemetryManager telemetryM;
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
    public PathChain HumanToLaunch;
    private final Pose start = new Pose(55.700, 8.740, Math.toRadians(180));
    private final Pose launchHuman = new Pose(60.540, 23, Math.toRadians(-154));
    private final Pose launchHuman2 = new Pose(60.540, 23, Math.toRadians(-154));
    private final Pose grab1 = new Pose(35, 36.5, Math.toRadians(180));
    private final Pose pickup1 = new Pose(9.5, 36.5, Math.toRadians(180));
    private final Pose grab2 = new Pose(38, 58.5, Math.toRadians(180));
    private final Pose pickup2 = new Pose(9, 58.5, Math.toRadians(180));
    private final Pose launchMiddle = new Pose(50.700, 87.40, Math.toRadians(-153));
    private final Pose grabHuman = new Pose(32, 11.5, Math.toRadians(180));
    private final Pose pickupHuman = new Pose(9, 11.5, Math.toRadians(180));
    private final Pose grabGate = new Pose(38, 87);
    private final Pose launchGate = new Pose(45, 87);
    private double farL = 1680;
    private double nearL = 1370;

    public void buildPaths() {
        preload = follower.pathBuilder()
                .addPath(
                        new BezierLine(start, launchHuman)
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), launchHuman.getHeading())
//                .addParametricCallback(0.00001, () -> {
//                    launcher.spin(farL);
//                })
                .build();

        Grab1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(launchHuman, grab1)
                )
                .setLinearHeadingInterpolation(launchHuman.getHeading(), grab1.getHeading())
                .build();

        Pickup1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(grab1, pickup1)
                )
                .setLinearHeadingInterpolation(grab1.getHeading(), pickup1.getHeading())
                .build();

        LaunchHuman = follower.pathBuilder()
                .addPath(
                        new BezierLine(grab1, launchHuman)
                )
                .setLinearHeadingInterpolation(pickup1.getHeading(), launchHuman2.getHeading())
//                .addParametricCallback(0.00001, () -> {
//                    launcher.spin(farL);
//                })
                .build();

        Grab2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(launchHuman, grab2)
                )
                .setLinearHeadingInterpolation(launchHuman2.getHeading(), grab2.getHeading())
                .build();
        Pickup2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(grab2, pickup2)
                )
//                .addParametricCallback(0.00001, () -> {
//                    launcher.spin(nearL);
//                })
                .setLinearHeadingInterpolation(grab2.getHeading(), pickup2.getHeading())
                .build();

        LaunchMiddle = follower.pathBuilder()
                .addPath(
                        new BezierLine(grab2, launchMiddle)
                )
                .setLinearHeadingInterpolation(grab2.getHeading(), Math.toRadians(-143))
//                .addParametricCallback(0.00001, () -> {
//                    launcher.spin(nearL);
//                })
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

        GrabHuman = follower.pathBuilder()
                .addPath(
                        new BezierLine(launchHuman, grabHuman)
                )
//                .addParametricCallback(0.00001, () -> {
//                    launcher.spin(nearL);
//                })
                .setLinearHeadingInterpolation(launchHuman2.getHeading(), Math.toRadians(180))
                .build();
        PickupHuman = follower.pathBuilder()
                .addPath(
                        new BezierLine(grabHuman, pickupHuman)
                )
//                .addParametricCallback(0.00001, () -> {
//                    launcher.spin(nearL);
//                })
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        HumanToLaunch = follower.pathBuilder()
                .addPath(
                        new BezierLine(pickupHuman, launchHuman2)
                )
//                .addParametricCallback(0.00001, () -> {
//                    launcher.spin(nearL);
//                })
                .setLinearHeadingInterpolation(pickupHuman.getHeading(), launchHuman2.getHeading())
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
        sensor = new ColorSensorSubsystem(hardwareMap);

        init = new Init(palete, onofrei);

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
                        new FollowPathCommand(follower, preload, true, 1),
                        new LaunchMotifBalls(robotStorage, telemetryM, palete, onofrei, launcher, 1680)
                ),
                new FollowPathCommand(follower, Grab1, true, 1),
                new ParallelCommandGroup(
                        new IntakeBall(robotStorage, telemetryM, intake, palete, sensor),
                        new FollowPathCommand(follower, Pickup1, true, 0.3)
                ),
                new FollowPathCommand(follower, LaunchHuman, true, 1),
                new LaunchMotifBalls(robotStorage, telemetryM, palete, onofrei, launcher, 1680),
//                new FollowPathCommand(follower, Grab2,true, 1),
                new FollowPathCommand(follower, GrabHuman, true, 1),
                new ParallelCommandGroup(
                        new IntakeBall(robotStorage, telemetryM, intake, palete, sensor),
                        new FollowPathCommand(follower, PickupHuman, true, 0.3)
                ),
                new FollowPathCommand(follower, HumanToLaunch, true, 1),
                new LaunchMotifBalls(robotStorage, telemetryM, palete, onofrei, launcher, 1680),
                new InstantCommand(
                        () -> launcher.brake()
                )
        );
        schedule(autonomousSequence);
    }

    @Override
    public void run() {
        super.run();
        follower.update();

        telemetryM.addData("Loop Time:", loopTime.milliseconds());
        telemetryM.addData("X:", follower.getPose().getX());
        telemetryM.addData("Y:", follower.getPose().getY());
        telemetryM.addData("Heading:", follower.getPose().getHeading());
        telemetryM.update(telemetry);

        loopTime.reset();
    }
}






