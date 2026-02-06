package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class LimelightSubsystem extends SubsystemBase {
    public static final int RED_APRILTAG_PIPELINE = 1,
                      BLUE_APRILTAG_PIPELINE = 0;
    private final Limelight3A limelight;

    public LimelightSubsystem(HardwareMap hwMap, int pipeline){
        this.limelight = hwMap.get(Limelight3A.class, "limelight");
        this.limelight.start();
        this.limelight.pipelineSwitch(pipeline);
    }

    @Override
    public void periodic() {
        // reconnect if disconnected
        if (!limelight.isRunning()) {
            limelight.stop();
            limelight.start();
        }
    }

    public void switchPipeline(int pipelineIndex) {
        limelight.pipelineSwitch(pipelineIndex);
    }

    public int getAprilTagId() {
        LLResult llResult = limelight.getLatestResult();
        if (llResult.isValid()) {
            for (FiducialResult tag : llResult.getFiducialResults()) {
                if (tag.getFiducialId() == 21 || tag.getFiducialId() == 22 || tag.getFiducialId() == 23)
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

    public void updateRobotHeading(double headingDegrees) {
        limelight.updateRobotOrientation(headingDegrees);
    } // mereu apelam asta inainte de getMegaTag2Pose()

    public Pose3D getMegaTag1Pose() {
        LLResult llResult = limelight.getLatestResult();
        if (llResult.isValid()) {
            return limelight.getLatestResult().getBotpose();
        } else {
            return null;
        }
    }

    public Pose3D getMegaTag2Pose() {
        LLResult llResult = limelight.getLatestResult();
        if (llResult.isValid()) {
            return limelight.getLatestResult().getBotpose_MT2();
        } else {
            return null;
        }
    }

}
