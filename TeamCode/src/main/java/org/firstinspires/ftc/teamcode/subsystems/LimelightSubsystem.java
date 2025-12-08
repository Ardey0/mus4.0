package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
    private final Limelight3A limelight;
    public LimelightSubsystem(HardwareMap hwMap){
        this.limelight = hwMap.get(Limelight3A.class, "limelight");
        this.limelight.start();
        this.limelight.pipelineSwitch(9);
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
}
