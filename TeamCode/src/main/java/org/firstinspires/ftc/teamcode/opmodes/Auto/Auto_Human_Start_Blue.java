package org.firstinspires.ftc.teamcode.opmodes.Auto;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class Auto_Human_Start_Blue extends OpMode {

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
        private final Pose start = new Pose(55.700, 8.740);
        private final Pose launchHuman = new Pose(58.540, 23.500);
        private final Pose grabHuman = new Pose(41.96, 37.750);
        private final Pose grabMiddle = new Pose(41.96, 62.5);
        private final Pose launchMiddle = new Pose(55.700, 8.740);
        private final Pose grabGate = new Pose(41.96, 87);
        private final Pose launchGate = new Pose(45, 87);

        public void buildPaths()
        {
            preload = follower.pathBuilder()
                    .addPath(
                            new BezierLine(start, launchHuman)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-160))
                    .build();

            GrabHuman = follower.pathBuilder()
                    .addPath(
                            new BezierLine(launchHuman,grabHuman)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-160), Math.toRadians(180))
                    .build();

            LaunchHuman = follower.pathBuilder()
                    .addPath(
                            new BezierLine(grabHuman,launchHuman)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-160))
                    .build();

            GrabMiddle = follower.pathBuilder()
                    .addPath(
                            new BezierLine(launchHuman,grabMiddle)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-160), Math.toRadians(180))
                    .build();

            LaunchMiddle = follower.pathBuilder()
                    .addPath(
                            new BezierLine(grabMiddle,launchMiddle)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-135))
                    .build();

            GrabGate = follower.pathBuilder()
                    .addPath(
                            new BezierLine(launchMiddle,grabGate)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-135), Math.toRadians(180))
                    .build();

            LaunchGate = follower.pathBuilder()
                    .addPath(
                            new BezierLine(grabGate,launchGate)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-135))
                    .build();
        }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(start);
    }

    @Override
    public void init_loop() {}

    public void autonomousPathUpdate()
    {
        switch (pathState) {
            case 0:
                follower.followPath(preload, 1, true);
                setPathState(-1);
                break;




        }
    }

    @Override
    public void loop()
    {
        follower.update();
        autonomousPathUpdate();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

}
