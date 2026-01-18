package org.firstinspires.ftc.teamcode.commands;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.RobotStorage;

import java.util.function.Supplier;

public class TurnToGoalBlue extends CommandBase {
    private final Follower follower;
    private PathChain pathChain;
    public TurnToGoalBlue(Follower follower) {
        this.follower = follower;
    }

    @Override
    public void initialize() {
        double x = follower.getPose().getX(), y = follower.getPose().getY();
        double targetHeadingRad = -Math.PI + Math.atan(x / (RobotStorage.fieldLengthIn - y)) +
                Math.asin(RobotStorage.centerToRampIn / Math.sqrt((RobotStorage.fieldLengthIn - y) * (RobotStorage.fieldLengthIn - y) + x * x));
        pathChain = follower.pathBuilder()
                .addPath(new BezierPoint(new Pose(follower.getPose().getX(), follower.getPose().getY(), targetHeadingRad)))
                .setConstantHeadingInterpolation(targetHeadingRad)
                .build();
        follower.followPath(pathChain);
//        follower.turnTo(targetHeadingRad);
    }

    @Override
    public void execute() {
//        follower.followPath(pathChain);
//        telemetry.addData("target heading deg", Math.toDegrees(targetHeadingRad));
    }

    @Override
    public void end(boolean interrupted) {
        follower.startTeleOpDrive(true);
    }

    @Override
    public boolean isFinished() {
        return false;
//        return !follower.isBusy();
    }

}
