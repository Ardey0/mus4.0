package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.CommandBase;

public class Relocalize extends CommandBase {
    private final Follower follower;

    public Relocalize(Follower follower) {
        this.follower = follower;
    }

    @Override
    public void initialize() {
        follower.setPose(new Pose(34, 133.4, Math.toRadians(-90)));
    }
}
