package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

/**
 * Improved AutoClose:
 * - Waits for a stable tag reading
 * - Backs up until smoothed distance > targetDist
 * - Ensures shooter spin-up (checks velocity) before feeding
 * - Safety timeouts everywhere
 */
@Autonomous(name="AutoClose1")
public class AutoClose1 extends LinearOpMode {

    // Drive motors
    private DcMotor LB, LF, RB, RF;

    // Vision
    private AprilTag vision;

    // Shooter + Intake
    private DcMotorEx LSX, RSX, IntakeEx;

    // Constants
    private static final double DRIVE_FWD = 0.5;
    private static final double SHOOTER_RPM = 1450;        // desired target rpm (adjust as needed)
    private static final double INTAKE_RPM1 = 1350;
    private static final double INTAKE_RPM2 = 800;

    // Distance target (stop when smoothed distance > targetDist inches)
    private static final double TARGET_DIST_IN = 65.0;

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

        // === SET RUN MODE FOR VELOCITY CONTROL ===
        LSX.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RSX.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        IntakeEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // === MAX BRAKING ENABLED ===
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Initializing AprilTag Vision...");
        telemetry.update();

        // Camera system
        vision = new AprilTag(hardwareMap, telemetry);
        telemetry.addLine("Vision Ready!");
        telemetry.addLine("Press PLAY to begin");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // =====================================================
        // PHASE A — WAIT FOR A STABLE TAG (short timeout)
        // =====================================================
        telemetry.addLine("Phase A: waiting for stable tag...");
        telemetry.update();
        double startA = getRuntime();
        double timeoutA = 4.0; // seconds max to wait for stable detection

        while (opModeIsActive()) {
            vision.update();
            AprilTagDetection tag = vision.getLatestTag();
            vision.addTelemetry();
            vision.addDashboardTelemetry(tag);
            telemetry.update();

            if (tag != null && vision.isStable()) {
                telemetry.addLine("TAG STABLE — applying brake pulse");
                telemetry.update();

                // brief active reverse pulse opposite of backing up to settle
                drive(+0.4, +0.4, +0.4, +0.4);
                sleep(120);
                drive(0,0,0,0);
                sleep(150);
                break;
            }

            if (getRuntime() - startA > timeoutA) {
                telemetry.addLine("Timeout waiting for stable tag — proceeding cautiously");
                telemetry.update();
                break;
            }

            // small sleep to keep loop reasonable
            sleep(30);
        }

        // =====================================================
        // PHASE B — BACK UP UNTIL SMOOTHED DISTANCE > TARGET_DIST_IN
        // =====================================================
        telemetry.addLine("Phase B: backing up until distance > " + TARGET_DIST_IN + " in");
        telemetry.update();

        double startB = getRuntime();
        double timeoutB = 6.0; // seconds max to back up
        while (opModeIsActive()) {

            vision.update();
            AprilTagDetection tag = vision.getLatestTag();

            double smoothedDist = vision.getSmoothedDistanceInches();

            // If we have a valid smoothed reading, act on it. Otherwise back up slowly trying to reacquire.
            if (smoothedDist > 0) {
                telemetry.addData("SmoothedDist(in)", "%.2f", smoothedDist);
                if (smoothedDist > TARGET_DIST_IN) {
                    drive(0,0,0,0);
                    telemetry.addLine("Reached target distance (smoothed).");
                    telemetry.update();
                    break;
                } else {
                    // continue backing up
                    drive(-DRIVE_FWD, -DRIVE_FWD, -DRIVE_FWD, -DRIVE_FWD);
                }
            } else {
                // no smoothed reading yet — back up slowly to try to find tag
                drive(-0.35, -0.35, -0.35, -0.35);
            }

            // Dashboard + driver telemetry
            vision.addTelemetry();
            vision.addDashboardTelemetry(tag);
            telemetry.update();

            if (getRuntime() - startB > timeoutB) {
                telemetry.addLine("Backup timeout reached — proceeding to shooting phase");
                telemetry.update();
                drive(0,0,0,0);
                break;
            }

            sleep(30);
        }

        // ensure stopped before shooting
        drive(0,0,0,0);
        sleep(100);

        // =====================================================
        // SHOOTER ROUTINE — spin up and check velocity
        // =====================================================
        telemetry.addLine("Spinning shooter...");
        telemetry.update();

        LSX.setVelocity(SHOOTER_RPM);
        RSX.setVelocity(SHOOTER_RPM);

        // Wait for shooter to converge or timeout
        double shooterStart = getRuntime();
        double shooterTimeout = 5.0; // seconds
        boolean shooterReady = false;

        while (opModeIsActive() && (getRuntime() - shooterStart < shooterTimeout)) {
            // If DcMotorEx supports getVelocity(), we can check it
            try {
                double vLS = Math.abs(LSX.getVelocity());
                double vRS = Math.abs(RSX.getVelocity());
                telemetry.addData("LS vel", "%.0f", vLS);
                telemetry.addData("RS vel", "%.0f", vRS);

                // require both wheels to be >= 90% of target
                if (vLS >= 0.90 * SHOOTER_RPM && vRS >= 0.90 * SHOOTER_RPM) {
                    shooterReady = true;
                    break;
                }
            } catch (Exception e) {
                // getVelocity may not be available on some controllers — fallback to timed spin-up
            }

            telemetry.update();
            sleep(150);
        }

        // Final guard: if we couldn't validate velocity, wait a short additional fixed time
        if (!shooterReady) {
            telemetry.addLine("Shooter not validated by velocity read; using time-based spin-up");
            telemetry.update();
            sleep(1500);
        } else {
            telemetry.addLine("Shooter ready");
            telemetry.update();
            sleep(200);
        }

        // Run intake cycles to feed rings
        telemetry.addLine("Feeding...");
        telemetry.update();

        IntakeEx.setVelocity(INTAKE_RPM1);
        sleep(1500);
        IntakeEx.setVelocity(0);
        sleep(200);
        IntakeEx.setVelocity(INTAKE_RPM2);
        sleep(1500);
        IntakeEx.setVelocity(0);

        // Stop shooter
        LSX.setVelocity(0);
        RSX.setVelocity(0);

        // Final tidy up
        stopAll();
        telemetry.addLine("AutoClose complete");
        telemetry.update();
    }

    private void stopAll() {
        LF.setPower(0);
        LB.setPower(0);
        RF.setPower(0);
        RB.setPower(0);
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
