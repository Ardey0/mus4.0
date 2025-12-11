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
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.commands.Init;
import org.firstinspires.ftc.teamcode.commands.IntakeBall;
import org.firstinspires.ftc.teamcode.commands.LaunchAllBalls;
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

    private LauncherSubsystem launcher;
    private PaleteSubsytem palete;
    private OnofreiSubsystem onofrei;
    private IntakeSubsystem intake;
    private ColorSensorSubsystem sensor;
    private RobotStorage robotStorage;
    private LimelightSubsystem limelight;

    private Init init;
    private ReadMotif readMotif;

    private ElapsedTime loopTime;
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
        CommandScheduler.getInstance().setBulkReading(hardwareMap, LynxModule.BulkCachingMode.MANUAL);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        loopTime = new ElapsedTime();

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
                new FollowPathCommand(follower, preload),
                new LaunchAllBalls(robotStorage, telemetryM, palete, onofrei, launcher, limelight),
                new FollowPathCommand(follower, GrabHuman),
                new ParallelCommandGroup(
                        new IntakeBall(robotStorage, telemetryM, intake, palete, sensor),
                        new FollowPathCommand(follower, PickupHuman, true, 0.2)
                ),
                new FollowPathCommand(follower, LaunchHuman),
                new LaunchAllBalls(robotStorage, telemetryM, palete, onofrei, launcher, limelight),
                new FollowPathCommand(follower, GrabMiddle),
                new ParallelCommandGroup(
                        new IntakeBall(robotStorage, telemetryM, intake, palete, sensor),
                        new FollowPathCommand(follower, PickupMiddle, true, 0.2)
                ),
                new FollowPathCommand(follower, LaunchMiddle),
                new LaunchAllBalls(robotStorage, telemetryM, palete, onofrei, launcher, limelight)
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
