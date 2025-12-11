package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.util.InterpLUT;

public class RobotStorage {
    private int[] roata = new int[3], motif = new int[3];
    private final InterpLUT distanceToLauncherSpeed = new InterpLUT() {{
        add(1.670, 1340);
        add(1.800, 1370);
        add(2.270, 1450);
        add(2.500, 1490);
        add(2.700, 1540);
        add(3.180, 1550);
        add(3.540, 1670);
        add(3.910, 1700);
        add(4.520, 1810);
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
        if (distance > 1.67 && distance < 4.52) {
            return distanceToLauncherSpeed.get(distance);
        } else return 0;
    }

}
