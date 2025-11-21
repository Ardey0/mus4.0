package org.firstinspires.ftc.teamcode.subsystems;

import java.util.HashMap;
import java.util.Map;

public class RoataSubsystem {
    private static final RoataSubsystem roata = new RoataSubsystem();

    // vars
    String[] sector = new String[3];

    private RoataSubsystem() {
        // def values
    }

    public static RoataSubsystem getInstance() {
        return roata;
    }

    public void setSector(int index, String culoare) {
        this.sector[index] = culoare;
    }

    public String getSector(int index) {
        return sector[index];
    }

    public int getNextFreeSector() {
        for (int i = 0; i < 3; i++) {
            if (sector[i] == null) {
                return i;
            }
        }
        return -1;
    }

}
