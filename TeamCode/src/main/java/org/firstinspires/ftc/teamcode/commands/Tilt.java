package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.subsystems.TiltSubsystem.DOWN;
import static org.firstinspires.ftc.teamcode.subsystems.TiltSubsystem.UP;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.TiltSubsystem;

public class Tilt extends CommandBase {
    private final TiltSubsystem tilt;

    public Tilt(TiltSubsystem tiltSubsystem) {
        this.tilt = tiltSubsystem;

        addRequirements(tilt);
    }

    public void initialize() {
        tilt.setPosition(UP);
    }

    public void end(boolean interrupted) {
        tilt.setPosition(DOWN);
    }
}
