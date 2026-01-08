package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.RobotStorage;

public class TurnToGoalBlue extends CommandBase {
    private final Follower follower;

    public TurnToGoalBlue(Follower follower) {
        this.follower = follower;
    }

    @Override
    public void initialize() {
        double x = follower.getPose().getX(), y = follower.getPose().getY();
        double targetHeadingRad = -Math.PI + Math.atan(x / (RobotStorage.fieldLengthIn - y)) +
                Math.asin(RobotStorage.centerToRampIn / Math.sqrt((RobotStorage.fieldLengthIn - y) * (RobotStorage.fieldLengthIn - y) + x * x));
        follower.turnTo(targetHeadingRad);
    }

    @Override
    public void end(boolean interrupted) {
        follower.startTeleOpDrive(true);
    }

    @Override
    public boolean isFinished() {
        return !follower.isBusy();
    }

}
