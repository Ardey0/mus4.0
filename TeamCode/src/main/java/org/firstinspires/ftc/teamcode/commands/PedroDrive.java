package org.firstinspires.ftc.teamcode.commands;


import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

public class PedroDrive extends CommandBase {
    private final Follower follower;
    private final GamepadEx gamepad;
    private final TelemetryManager telemetry;

    public PedroDrive(TelemetryManager telemetryManager, GamepadEx gamepad, Follower follower) {
        this.follower = follower;
        this.gamepad = gamepad;
        this.telemetry = telemetryManager;
    }

    @Override
    public void initialize() {
        follower.startTeleOpDrive();
    }

    @Override
    public void execute() {
        follower.setTeleOpDrive(
                -gamepad.getLeftY(),
                -gamepad.getLeftX(),
                -gamepad.getRightX(),
                true
        ); // Robot Centric

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
    }

}
