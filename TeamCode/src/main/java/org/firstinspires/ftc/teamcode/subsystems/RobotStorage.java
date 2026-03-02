package org.firstinspires.ftc.teamcode.subsystems;

import static com.seattlesolvers.solverslib.util.MathUtils.clamp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

public class RobotStorage {
    private int[] roata = new int[3], motif = new int[3];
    public static final double centerToRampIn = 2.83, fieldLengthIn = 144;
    private static final double INCH_TO_METERS = 1.0 / 39.37007874;
    private static Pose autoEnd = null;
    private double distanceToDepotM = 0;

    public RobotStorage() {
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

    public double getLauncherSpeedForDist() {
        double d2 = distanceToDepotM * distanceToDepotM;
        double d3 = d2 * distanceToDepotM;
        double d4 = d3 * distanceToDepotM;
        return clamp(-9.88535 * d4 +
                        119.79707 * d3 -
                        518.17132 * d2 +
                        1120.16708 * distanceToDepotM + 365.28292,
                0, 2050);
    }

    public double getRampAngleForDist() {
        double d2 = distanceToDepotM * distanceToDepotM;
        double d3 = d2 * distanceToDepotM;
        double d4 = d3 * distanceToDepotM;
        return clamp(-0.00173126 * d4 +
                        0.0437586 * d3 -
                        0.360518 * d2 +
                        1.29682 * distanceToDepotM - 0.848703,
                0, 0.92);
    }

    public void updateAutoEndPose(Pose endPose) {
        autoEnd = endPose;
    }

    public Pose getAutoEndPose() {
        return autoEnd;
    }

    public void updateDistanceToGoal(double x, double y, int alliance) {
        double dx = (alliance == 0) ? x : (144.0 - x), dy = 144.0 - y;
        distanceToDepotM = Math.sqrt(dx * dx + dy * dy) * INCH_TO_METERS;
    }

    public void updateDistanceToGoal(Follower follower, int alliance) {
        double x = follower.getPose().getX(), y = follower.getPose().getY();
        double dx = (alliance == 0) ? x : (144.0 - x), dy = 144.0 - y;
        distanceToDepotM = Math.sqrt(dx * dx + dy * dy) * INCH_TO_METERS;
    }

    public double getDistanceToDepotM() {
        return distanceToDepotM;
    }
}
