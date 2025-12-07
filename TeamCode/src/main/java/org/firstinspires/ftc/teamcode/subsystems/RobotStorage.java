package org.firstinspires.ftc.teamcode.subsystems;

public class RobotStorage {
    private int[] roata = new int[3], motif = new int[3];

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

}
