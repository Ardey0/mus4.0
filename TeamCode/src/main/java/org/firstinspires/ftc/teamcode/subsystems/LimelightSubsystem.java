package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class LimelightSubsystem extends SubsystemBase {
    public static final int RED_APRILTAG_PIPELINE = 9,
                      BLUE_APRILTAG_PIPELINE = 8;
    private final Limelight3A limelight;
    public LimelightSubsystem(HardwareMap hwMap, int pipeline){
        this.limelight = hwMap.get(Limelight3A.class, "limelight");
        this.limelight.start();
        this.limelight.pipelineSwitch(pipeline);
    }

    public int getAprilTagId() {
        LLResult llResult = limelight.getLatestResult();
        if (llResult.isValid()) {
            for (FiducialResult tag : llResult.getFiducialResults()) {
                return tag.getFiducialId();
            }
        }
        return 0;
    }

    public double getTx() {
        LLResult llResult = limelight.getLatestResult();
        if (llResult.isValid()) {
            return llResult.getTx();
        }
        return 0;
    }

    public double getDistanceToDepot() {
        LLResult llResult = limelight.getLatestResult();
        double straightLineDistance = -1;
        for (LLResultTypes.FiducialResult tag : llResult.getFiducialResults()) {
            if (tag.getFiducialId() == 20 || tag.getFiducialId() == 24) {
                Pose3D targetPose = tag.getTargetPoseCameraSpace();
                double xCam = targetPose.getPosition().x;
                double yCam = targetPose.getPosition().y;
                double zCam = targetPose.getPosition().z;

                straightLineDistance = Math.sqrt(
                        Math.pow(xCam, 2) +
                        Math.pow(yCam, 2) +
                        Math.pow(zCam, 2)
                );
            }
        }

        return straightLineDistance;
    }
}
