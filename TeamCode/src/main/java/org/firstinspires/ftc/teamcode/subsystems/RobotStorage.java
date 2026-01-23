package org.firstinspires.ftc.teamcode.subsystems;

import static com.seattlesolvers.solverslib.util.MathUtils.clamp;

import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.util.InterpLUT;

public class RobotStorage {
    private int[] roata = new int[3], motif = new int[3];
    public static final double centerToRampIn = 2.83, fieldLengthIn = 144;
    private static Pose autoEnd = null;

    private final InterpLUT distanceToRampAngle = new InterpLUT() {{
//        add(0.95, 0);
//        add(1.2, 0.15);
//        add(1.3, 0.2);
//        add(1.47, 0.28);
//        add(1.84, 0.4);
//        add(2, 0.39);
//        add(2.26, 0.42);
//        add(2.52, 0.43);
//        add(2.81, 0.43);
//        add(3.47,0.43);
//        add(3.87, 0.43);
        // set 2
//        add(0.95, 0.1);
//        add(1.1, 0.15);
//        add(1.25, 0.21);
//        add(1.47, 0.28);
//        add(1.84, 0.4);
//        add(2, 0.41);
//        add(2.26, 0.43);
//        add(2.52, 0.47);
//        add(2.8, 0.47);
//        add(3.48,0.51);
//        add(3.87, 0.515);
        add(0.9, 0);
        add(1.43, 0.32);
        add(1.7, 0.5);
        add(2.02, 0.58);
        add(2.29, 0.75);
        add(2.52,0.77);
        add(2.8, 0.87);
        add(3.38, 0.9);
        add(3.72, 0.92);
        add(3.94, 0.93);
    }};

    private final InterpLUT distanceToLauncherSpeed = new InterpLUT() {{
        // set 2
//        add(0.95, 1400);
//        add(1.1, 1300);
//        add(1.47, 1300);
//        add(1.83, 1400);
//        add(2, 1490);
//        add(2.26, 1500);
//        add(2.52, 1580);
//        add(2.8, 1610);
//        add(3.46, 1750);
//        add(3.87, 1830);
        // set 1
//        add(0.95, 1320);
//        add(1.2, 1300);
//        add(1.47, 1310);
//        add(1.84, 1380);
//        add(2, 1430);
//        add(2.26, 1500);
//        add(2.52, 1550);
//        add(2.81, 1650);
//        add(3.47, 1820);
//        add(3.87, 1880);
        add(0.9, 1300);
        add(1.43, 1440);
        add(1.7, 1500);
        add(2.02, 1600);
        add(2.29, 1670);
        add(2.52, 1750);
        add(2.8, 1780);
        add(3.38, 1870);
        add(3.72, 1970);
        add(3.94, 2020);
    }};

    public RobotStorage() {
        distanceToLauncherSpeed.createLUT();
        distanceToRampAngle.createLUT();
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

    public int[] getMotif() {
        return motif;
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

    public double getLauncherSpeedForCoordsBlue(double x, double y) {
        double distanceM = Math.sqrt((-x) * (-x) + (144 - y) * (144 - y)) / 39.37007874;
        return distanceToLauncherSpeed.get(clamp(distanceM, 0.95, 3.86));
    }

    public double getLauncherSpeedForCoordsRed(double x, double y) {
        double distanceM = Math.sqrt((144 - x) * (144 - x) + (144 - y) * (144 - y)) / 39.37007874;
        return distanceToLauncherSpeed.get(clamp(distanceM, 0.95, 3.86));
    }

    public double getLauncherSpeedForCoordsNew(double x, double y) {
        double distanceM = Math.sqrt((-x) * (-x) + (144 - y) * (144 - y)) / 39.37007874;
        return clamp(23.10907 * Math.pow(distanceM, 4) -
                219.29832 * Math.pow(distanceM, 3) +
                707.64469 * Math.pow(distanceM, 2) -
                654.41708 * distanceM + 1462.18186,
                0, 2050);
    }

    public double getRampAngleForCoordsBlue(double x, double y) {
        double distanceM = Math.sqrt((-x) * (-x) + (144 - y) * (144 - y)) / 39.37007874;
        if (distanceM < 0.95) {
            return 0;
        } else {
            return distanceToRampAngle.get(clamp(distanceM, 0.95, 3.86));
        }
    }

    public double getRampAngleForCoordsRed(double x, double y) {
        double distanceM = Math.sqrt((144 - x) * (144 - x) + (144 - y) * (144 - y)) / 39.37007874;
        if (distanceM < 0.95) {
            return 0;
        } else {
            return distanceToRampAngle.get(clamp(distanceM, 0.95, 3.86));
        }
    }

    public double getRampAngleForCoordsNew(double x, double y) {
        double distanceM = Math.sqrt((-x) * (-x) + (144 - y) * (144 - y)) / 39.37007874;
        return clamp(0.0143789 * Math.pow(distanceM, 4) -
                        0.131801 * Math.pow(distanceM, 3) +
                        0.289158 * Math.pow(distanceM, 2) +
                        0.38897 * distanceM - 0.496897,
                0, 0.95);
    }

    public void setAutoEndPose(Pose endPose) {
        autoEnd = endPose;
    }

    public Pose getAutoEndPose() {
        return autoEnd;
    }

}
