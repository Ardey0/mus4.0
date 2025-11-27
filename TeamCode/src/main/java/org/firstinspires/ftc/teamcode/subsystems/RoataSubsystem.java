package org.firstinspires.ftc.teamcode.subsystems;

import java.util.Objects;

public class RoataSubsystem {
    // vars
    String[] sector = new String[3];

    public RoataSubsystem() {
        // def values
        sector[0] = null;
        sector[1] = null;
        sector[2] = null;
    }

    public void setSector(int index, String culoare) {
        sector[index] = culoare;
    }

    public String getSectorColor(int index) {
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

    public int getNextSectorWithColor(String color) {
        for (int i = 0; i < 3; i++) {
            if (sector[i].equals(color)) {
                return i;
            }
        }
        return -1;
    }

}
