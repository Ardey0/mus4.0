package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.OnofreiSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PaleteSubsytem;

public class PlaceBall extends CommandBase {
    private final PaleteSubsytem palete;
    private final OnofreiSubsystem onofrei;
    private final int sector;

    public PlaceBall(PaleteSubsytem paleteSubsytem, OnofreiSubsystem onofreiSubsystem, int sector) {
        this.palete = paleteSubsytem;
        this.onofrei = onofreiSubsystem;
        this.sector = sector;

        addRequirements(palete, onofrei);
    }

    public void initialize() {
        onofrei.setPosition(OnofreiSubsystem.IN);
        switch (sector) {
            case 1:
                palete.setPosition(PaleteSubsytem.IN_BILA_1);
                break;
            case 2:
                palete.setPosition(PaleteSubsytem.IN_BILA_2);
                break;
            case 3:
                palete.setPosition(PaleteSubsytem.IN_BILA_3);
                break;
            default:
                throw new IllegalArgumentException("Invalid sector: " + sector);
        }
    }

    public boolean isFinished() {
        return true;
    }
}
