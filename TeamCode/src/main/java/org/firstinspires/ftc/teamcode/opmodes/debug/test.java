package org.firstinspires.ftc.teamcode.opmodes.debug;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.util.InterpLUT;
import com.seattlesolvers.solverslib.util.LUT;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp
@Configurable
public class test extends LinearOpMode {
    TelemetryManager telemetryM;
    public static double onoPos = 0;
    public static double paletaPos;
    public static double viteza = 0;
    public static double kP = 0.0054, kI = 0, kD = 0.0000015, kF = 0.00036;  // lansator
    public static double motorPower = 0;

    public static InterpLUT lut = new InterpLUT()
    {{
        add(1.000, 1350); // orice unghi
        add(1.670, 1530); //
        add(1.800, 1570); // 2.5 deg
        add(2.100, 1600); // 2.5 deg
        add(2.455, 1670); /// 1.2 deg 90
        add(2.740, 1710); //
        add(3.210, 1800); // 3.1 deg
        add(3.730, 1900); // 4.4 deg
        add(3.910, 1920); // 5.8 deg
        add(4.200, 2030); // 5.4 deg
        add(4.420, 2100); // 1.6 deg
    }};

    double straightLineDistance;

    @Override
    public void runOpMode() throws InterruptedException {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "front_left");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "front_right");
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "back_left");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "back_right");
        DcMotorEx launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");

        ServoImplEx onofrei = hardwareMap.get(ServoImplEx.class, "onofrei");
        ServoImplEx paleta = hardwareMap.get(ServoImplEx.class, "palete");

        NormalizedColorSensor senzorTavan = hardwareMap.get(NormalizedColorSensor.class, "senzor_tavan");
        NormalizedColorSensor senzorGaura = hardwareMap.get(NormalizedColorSensor.class, "senzor_gaura");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        PIDFController controller = new PIDFController(kP, kI, kD, kF);

        boolean launcherState = false, intakeState = false, trackingAprilTag = false;

        ElapsedTime time = new ElapsedTime();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        limelight.start();
        limelight.pipelineSwitch(8);


        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        telemetryM.addLine("Initialized! Waiting for start...");
        telemetryM.update(telemetry);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            lut.createLUT();
            controller.setPIDF(kP, kI, kD, kF);
            time.reset();
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            // Store gamepad values from the previous loop iteration
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            // Store gamepad values from this loop iteration
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            //sistem de miscare
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;


            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            LLResult result = limelight.getLatestResult();

            for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                Pose3D targetPose = tag.getTargetPoseCameraSpace();
                double xCam = targetPose.getPosition().x;
                double yCam = targetPose.getPosition().y;
                double zCam = targetPose.getPosition().z;

                straightLineDistance = Math.sqrt(
                        Math.pow(xCam, 2) +
                        Math.pow(yCam, 2) +
                        Math.pow(zCam, 2)
                );
                telemetryM.addData("straightLineDistance", straightLineDistance);
                telemetryM.addData("camera", tag.getTargetPoseCameraSpace());
            }

            //Lansator
            if (previousGamepad1.b && !currentGamepad1.b) {
                launcherState = !launcherState;
            }
            if (launcherState) {
                double power = controller.calculate(launcher.getVelocity(), viteza);
                telemetryM.addData("viteza target:", viteza);
                telemetryM.addData("power:", power);
                launcher.setPower(power);
            } else {
                launcher.setPower(0);
            }

            //IntakeSubsystem
            if (previousGamepad1.a && !currentGamepad1.a) {
                intakeState = !intakeState;
            }
            if (intakeState) {
                intake.setPower(1);
            } else {
                intake.setPower(0);
            }

            //Onofrei
            onofrei.setPosition(onoPos);

            //Paleta
            paleta.setPosition(paletaPos);

            telemetryM.addData("senzor tavan:", ((DistanceSensor) senzorTavan).getDistance(DistanceUnit.CM));
            telemetryM.addData("senzor gaura:", ((DistanceSensor) senzorGaura).getDistance(DistanceUnit.CM));
            telemetryM.addData("tx:", limelight.getLatestResult().getTx());
            telemetryM.addData("rx", rx);
            telemetryM.addData("loop time:", time.milliseconds());
            telemetryM.addData("viteza lansator: ", launcher.getVelocity());
            telemetryM.addData("launcher on?", launcherState);
            telemetryM.update(telemetry);
        }
    }
}
