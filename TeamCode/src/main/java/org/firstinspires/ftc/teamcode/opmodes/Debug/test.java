package org.firstinspires.ftc.teamcode.opmodes.Debug;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp
@Config
public class test extends LinearOpMode {
    public static double onoPos = 0;
    public static double paletaPos;
    public static double viteza = 1450;
//    public static double kP = 0.004, kI = 0, kD = 0.0000007, kF = 0.000375;  // lansator
    public static double kP = 0.033, kI = 0, kD = 0.00001, kF = 0, kS = 0.16;  // camera, de tunat kS
    public static double motorPower = 0;

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

        limelight.start();
        limelight.pipelineSwitch(9);


        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        telemetry.addLine("Initialized! Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
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
            double rx;

            if (previousGamepad1.y && !currentGamepad1.y) {
                trackingAprilTag = !trackingAprilTag;
            }
            if (trackingAprilTag) {
                double power = controller.calculate(-limelight.getLatestResult().getTx(), 0);
                rx = power + kS * Math.signum(power);

            } else {
                rx = gamepad1.right_stick_x;
            }
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeft.setPower(motorPower);
            backLeft.setPower(motorPower);
            frontRight.setPower(-motorPower);
            backRight.setPower(-motorPower);
            
            //Lansator
            if (previousGamepad1.b && !currentGamepad1.b) {
                launcherState = !launcherState;
            }
            if (launcherState) {
                double power = controller.calculate(launcher.getVelocity(), viteza);
                telemetry.addData("power:", power);
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

            LLResult result = limelight.getLatestResult();

            for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                Pose3D targetPose = tag.getTargetPoseCameraSpace();
                double xCam = targetPose.getPosition().x;
                double yCam = targetPose.getPosition().y;
                double zCam = targetPose.getPosition().z;

                double straightLineDistance = Math.sqrt(
                        Math.pow(xCam, 2) +
                        Math.pow(yCam, 2) +
                        Math.pow(zCam, 2)
                );
                telemetry.addData("straightLineDistance", straightLineDistance);
                telemetry.addData("camera", tag.getTargetPoseCameraSpace());
            }
            telemetry.addData("tx:", limelight.getLatestResult().getTx());
            telemetry.addData("rx", rx);
            telemetry.addData("loop time:", time.milliseconds());
            telemetry.addData("viteza lansator: ", launcher.getVelocity());
            telemetry.addData("launcher on?", launcherState);
            telemetry.update();
        }
    }
}
