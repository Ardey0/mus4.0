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

    public int aprilTagId() {
        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            for (FiducialResult tag : result.getFiducialResults()) {
                return tag.getFiducialId();
            }
        }
        return 0;
    }

}
