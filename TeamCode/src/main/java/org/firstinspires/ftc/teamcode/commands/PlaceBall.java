package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.RoataSubsystem;

public class PlaceBall extends CommandBase {
    private final RoataSubsystem roata;
    private final int sector;

    public PlaceBall(RoataSubsystem roataSubsystem, int sector) {
        this.roata = roataSubsystem;
        this.sector = sector;

        addRequirements(roata);
    }

    public void initialize() {
        roata.setOnofreiPosition(0);
        switch (sector) {
            case 1:
                roata.setPaletePosition(0);
                break;
            case 2:
                roata.setPaletePosition(0.36);
                break;
            case 3:
                roata.setPaletePosition(0.71);
                break;
            default:
                throw new IllegalArgumentException("Invalid sector: " + sector);
        }
    }

    public boolean isFinished() {
        return true;
    }
}
