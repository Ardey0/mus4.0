package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.util.InterpLUT;

public class RobotStorage {
    private int[] roata = new int[3], motif = new int[3];
    private final InterpLUT distanceToLauncherSpeed = new InterpLUT() {{
        add(1.000, 1390); // orice unghi
        add(1.450, 1450);
        add(1.670, 1480); //
        add(1.800, 1530); // 2.5 deg
        add(2.200, 1610); // 2.5 deg
        add(2.455, 1670); /// 1.2 deg 90
        add(2.760, 1710); //
        add(3.210, 1800); // 3.1 deg
        add(3.730, 1950); // 4.4 deg
        add(3.910, 1970); // 5.8 deg
        add(4.200, 1990); // 5.4 deg
        add(4.420, 2020); // 1.6 deg
    }};

    public RobotStorage() {
        distanceToLauncherSpeed.createLUT();
        roata[0] = 0;
        roata[1] = 0;
        roata[2] = 0;
    }

    public void setMotif(int tagId) {
        switch (tagId) {
            case 21:
                this.motif[0] = 1;
                this.motif[1] = 2;
                this.motif[2] = 2;
                break;
            case 22:
                this.motif[0] = 2;
                this.motif[1] = 1;
                this.motif[2] = 2;
                break;
            case 23:
                this.motif[0] = 2;
                this.motif[1] = 2;
                this.motif[2] = 1;
                break;
        }
    }

    public void setSector(int index, int culoare) {
        roata[index] = culoare;
    }

    public int getSectorColor(int index) {
        return roata[index];
    }

    public int getNextFreeSector() {
        for (int i = 0; i < 3; i++) {
            if (roata[i] == 0) {
                return i;
            }
        }
        return -1;
    }

    public int getCurrentSector() {
        for (int i = 2; i >= 0; i--) {
            if (roata[i] != 0) {
                return i;
            }
        }
        return 0;
    }

    public int getNextSectorWithMotifBall(int motifIndex) {
        if (motifIndex > 2) {
            return -1;
        }
        for (int i = 0; i < 3; i++) {
            if (roata[i] == motif[motifIndex]) {
                return i;
            }
        }
        return -1;
    }

    public int getNextSectorWithColor(int color) {
        for (int i = 0; i < 3; i++) {
            if (roata[i] == color) {
                return i;
            }
        }
        return -1;
    }

    public double getLauncherSpeedForDistance(double distance) {
        if (distance > 1 && distance < 4.52) {
            return distanceToLauncherSpeed.get(distance);
        } else return 0;
    }

}
