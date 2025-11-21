package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name="AutoClose2")
public class AutoClose2 extends LinearOpMode {

    // Drive motors
    private DcMotor LB, LF, RB, RF;

    // Vision
    private AprilTag vision;

    // Shooter + Intake
    private DcMotorEx LSX, RSX, IntakeEx;

    // Constants
    private static final double DRIVE_FWD = 0.57;              // slightly increased
    private static final double SHOOTER_RPM = 1450;
    private static final double INTAKE_RPM1 = 1350;
    private static final double INTAKE_RPM2 = 800;

    // Stop earlier (65 → 66)
    private static final double TARGET_DIST_IN = 66.0;

    @Override
    public void runOpMode() throws InterruptedException {

        // === Hardware Map ===
        LB = hardwareMap.get(DcMotor.class, "LB");
        LF = hardwareMap.get(DcMotor.class, "LF");
        RB = hardwareMap.get(DcMotor.class, "RB");
        RF = hardwareMap.get(DcMotor.class, "RF");
        LSX = hardwareMap.get(DcMotorEx.class, "LS");
        RSX = hardwareMap.get(DcMotorEx.class, "RS");
        IntakeEx = hardwareMap.get(DcMotorEx.class, "Intake");

        // === Motor Directions ===
        LB.setDirection(DcMotor.Direction.FORWARD);
        LF.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.FORWARD);
        RF.setDirection(DcMotor.Direction.FORWARD);
        LSX.setDirection(DcMotorEx.Direction.REVERSE);
        RSX.setDirection(DcMotorEx.Direction.FORWARD);
        IntakeEx.setDirection(DcMotorEx.Direction.REVERSE);

        // Encoders
        LSX.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RSX.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        IntakeEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Brake
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Initializing AprilTag Vision...");
        telemetry.update();

        vision = new AprilTag(hardwareMap, telemetry);

        telemetry.addLine("Vision Ready!");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // =====================================================
        // PHASE A — WAIT FOR STABLE TAG
        // =====================================================
        double startA = getRuntime();
        while (opModeIsActive() && getRuntime() - startA < 4.0) {

            vision.update();
            AprilTagDetection tag = vision.getLatestTag();

            vision.addTelemetry();
            vision.addDashboardTelemetry(tag);
            telemetry.update();

            if (tag != null && vision.isStable()) {
                drive(+0.4, +0.4, +0.4, +0.4);
                sleep(120);
                drive(0,0,0,0);
                sleep(150);
                break;
            }

            sleep(30);
        }

        // =====================================================
        // PHASE B — BACK UP UNTIL DIST > TARGET
        // =====================================================
        double startB = getRuntime();

        while (opModeIsActive() && getRuntime() - startB < 6.0) {

            vision.update();
            AprilTagDetection tag = vision.getLatestTag();
            double smoothed = vision.getSmoothedDistanceInches();

            if (smoothed > 0) {
                if (smoothed > TARGET_DIST_IN) {
                    drive(0,0,0,0);
                    break;
                } else {
                    drive(-DRIVE_FWD, -DRIVE_FWD, -DRIVE_FWD, -DRIVE_FWD);
                }
            } else {
                drive(-0.35, -0.35, -0.35, -0.35);
            }

            vision.addTelemetry();
            vision.addDashboardTelemetry(tag);
            telemetry.update();

            sleep(30);
        }

        drive(0,0,0,0);
        sleep(100);

        // =====================================================
        // ** NEW ANTI-JAM PULSE **
        // =====================================================
        LSX.setPower(-0.2);
        RSX.setPower(-0.2);
        IntakeEx.setPower(-0.15);
        sleep(20);
        LSX.setPower(0);
        RSX.setPower(0);
        IntakeEx.setPower(0);
        sleep(80);

        // =====================================================
        // SHOOTER SPINUP
        // =====================================================
        LSX.setVelocity(SHOOTER_RPM);
        RSX.setVelocity(SHOOTER_RPM);

        double spinStart = getRuntime();
        boolean shooterReady = false;
        while (opModeIsActive() && getRuntime() - spinStart < 5.0) {
            double v1 = Math.abs(LSX.getVelocity());
            double v2 = Math.abs(RSX.getVelocity());

            telemetry.addData("LS vel", "%.0f", v1);
            telemetry.addData("RS vel", "%.0f", v2);
            telemetry.update();

            if (v1 >= 0.9 * SHOOTER_RPM && v2 >= 0.9 * SHOOTER_RPM) {
                shooterReady = true;
                break;
            }

            sleep(150);
        }

        if (!shooterReady) sleep(1500);
        else sleep(200);

        // =====================================================
        // FEED **3 RINGS**
        // =====================================================
        shootRing();
        sleep(200);

        shootRing();
        sleep(200);

        shootRing();

        // Stop shooter
        LSX.setVelocity(0);
        RSX.setVelocity(0);
        IntakeEx.setVelocity(0);

        // =====================================================
        // STRAFE RIGHT 5 INCHES
        // =====================================================
        strafeRightTimed(5);   // simple timed function

        stopAll();
    }

    private void shootRing() {
        IntakeEx.setVelocity(INTAKE_RPM1);
        sleep(600);   // shorter than auto1 to only shoot 1 ring
        IntakeEx.setVelocity(0);
    }

    private void strafeRightTimed(double inches) {
        double time = inches * 55;   // tune this based on robot
        drive(0.5, -0.5, -0.5, 0.5);
        sleep((long) time);
        drive(0,0,0,0);
    }

    private void stopAll() {
        drive(0,0,0,0);
        LSX.setVelocity(0);
        RSX.setVelocity(0);
        IntakeEx.setVelocity(0);
    }

    private void drive(double lf, double lb, double rf, double rb) {
        LF.setPower(lf);
        LB.setPower(lb);
        RF.setPower(rf);
        RB.setPower(rb);
    }
}
