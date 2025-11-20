package org.firstinspires.ftc.teamcode.OpModes.Teste;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ftc.localization.Encoder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

@TeleOp
@Config
public class TelemetrieEncodere extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        AnalogInput encoderx = hardwareMap.get(AnalogInput.class, "perp");
        AnalogInput encodery = hardwareMap.get(AnalogInput.class, "parp");
        while (opModeIsActive() && !isStopRequested()) {
            double x = encoderx.getVoltage() / 3.3 * 360;
            double y = encodery.getVoltage() / 3.3 * 360;
            telemetry.addData("x: ", x);
            telemetry.addData("y: ", y);
            telemetry.update();
        }
    }
}
