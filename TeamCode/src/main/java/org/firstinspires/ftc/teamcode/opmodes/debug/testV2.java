package org.firstinspires.ftc.teamcode.opmodes.debug;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.IntakeKickerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OnofreiSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PaleteSubsytem;
import org.firstinspires.ftc.teamcode.subsystems.RampaSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotStorage;
import org.firstinspires.ftc.teamcode.subsystems.SenzorGauraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SenzorRoataSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SenzorTavanSubsystem;

import java.util.List;

@Configurable
@TeleOp
public class testV2 extends LinearOpMode {
    public static double launcherSpeed = 0, rampAngle = 0, onoPos = 0, paletaPos = 0, intakePos = 0, intakePower = 0;

    private ElapsedTime loopTime = new ElapsedTime();
    private TelemetryManager telemetryM;

    private LauncherSubsystem launcher;
    private PaleteSubsytem palete;
    private OnofreiSubsystem onofrei;
    private IntakeSubsystem intake;
    private IntakeKickerSubsystem intakeKicker;
    private RampaSubsystem rampa;
    private SenzorTavanSubsystem senzorTavan;
    private SenzorRoataSubsystem senzorRoata;
    private SenzorGauraSubsystem senzorGaura;
    private RobotStorage robotStorage;
    private LimelightSubsystem limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "front_left");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "front_right");
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "back_left");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "back_right");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        launcher = new LauncherSubsystem(hardwareMap);
        palete = new PaleteSubsytem(hardwareMap);
        onofrei = new OnofreiSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        intakeKicker = new IntakeKickerSubsystem(hardwareMap);
        rampa = new RampaSubsystem(hardwareMap);
        senzorTavan = new SenzorTavanSubsystem(hardwareMap);
        senzorRoata = new SenzorRoataSubsystem(hardwareMap);
        senzorGaura = new SenzorGauraSubsystem(hardwareMap);
        limelight = new LimelightSubsystem(hardwareMap, LimelightSubsystem.BLUE_APRILTAG_PIPELINE);
        robotStorage = new RobotStorage();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            loopTime.reset();

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

            launcher.spin(launcherSpeed);
            intake.suck(intakePower);
            palete.setPosition(paletaPos);
            onofrei.setPosition(onoPos);
            intakeKicker.setPosition(intakePos);
            rampa.setPosition(rampAngle);

            telemetryM.addData("launcher speed", launcher.getVelocity());
            telemetryM.addData("loop time", loopTime);
            telemetryM.update();
        }
    }
}
