package org.firstinspires.ftc.teamcode;

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
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward;

import java.util.List;

@TeleOp
public class test extends LinearOpMode {

    public static double onoPos = 0;
    public static double paletaPos;
    public static double viteza = 1400, acceleratie = 150;
    public static double kV = 0.000336, kA = 0.0001, kS = 0.085;

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

        SimpleMotorFeedforward controller = new SimpleMotorFeedforward(kS, kV, kA);

        boolean launcherState = false, intakeState = false;

        ElapsedTime time = new ElapsedTime();
        ElapsedTime acceleratieLansator = new ElapsedTime();

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
            
            //Lansator
            if (previousGamepad1.a && !currentGamepad1.a) {
                launcherState = !launcherState;
                time.reset();
            }
            if (launcherState) {
                double power = controller.calculate(viteza, acceleratie);
                launcher.setPower(power);
//                launcher.setPower(putereLansator);
//                if(time.milliseconds()>=3000)
//                    onofrei.setPosition(1);
                acceleratieLansator.reset();
                telemetry.addData("power: ", power);
            } else {
                launcher.setPower(0);
//                onofrei.setPosition(0);
            }

            //IntakeSubsystem
            if (previousGamepad1.b && !currentGamepad1.b) {
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

            for (LLResultTypes.FiducialResult tag : result.getFiducialResults())
                telemetry.addData("id", tag.getFiducialId());
            telemetry.addData("viteza lansator: ", launcher.getVelocity());
            telemetry.addData("velocity error: ", Math.abs(launcher.getVelocity() - viteza));
            if (Math.abs(launcher.getVelocity() - viteza) < viteza * 0.02) {
                telemetry.addData("timp pana ajunge la viteza: ", acceleratieLansator.milliseconds());
//                launcher.setPower(0);
                telemetry.update();
//                sleep(10000);
            }
            telemetry.update();

        }

    }

}
