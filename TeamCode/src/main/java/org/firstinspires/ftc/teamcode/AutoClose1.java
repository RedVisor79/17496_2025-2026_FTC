package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name="AutoClose1")
public class AutoClose1 extends LinearOpMode {

    // Drive motors
    private DcMotor LB, LF, RB, RF;

    // Vision
    private AprilTag vision;

    // Shooter + Intake
    private DcMotorEx LSX, RSX, IntakeEx;

    // Constants
    private static final double DRIVE_FWD = .5;
    private static final double SHOOTER_RPM = 1450;
    private static final double INTAKE_RPM1 = 1350;
    private static final double INTAKE_RPM2 = 800;

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
        // PHASE A — BACK UP UNTIL TAG SEEN (MAX BRAKING)
        // =====================================================

        while (opModeIsActive()) {
            vision.update();
            AprilTagDetection tag = vision.getLatestTag();

            if (tag != null) {

                telemetry.addLine("TAG ACQUIRED — APPLYING MAX BRAKE!");
                telemetry.update();

                // === ACTIVE COUNTER FORCE BRAKE PULSE ===
                drive(+0.4, +0.4, +0.4, +0.4); // Opposite direction of backing up
                sleep(120);  // Brief strong pulse

                // === FULL STOP ===
                drive(0, 0, 0, 0);
                sleep(150);  // Stability pause

                telemetry.addLine("Robot Stopped — Entering Phase B");
                telemetry.update();

                break;
            }

            if (getRuntime() > 3.5) break;

            // Backup blindly
            drive(-DRIVE_FWD, -DRIVE_FWD, -DRIVE_FWD, -DRIVE_FWD);
            sleep(15);
        }

        drive(0, 0, 0, 0);
        sleep(50);

        // =====================================================
        // PHASE B — DRIVE UNTIL TAG DISTANCE > 65 IN
        // =====================================================
        while (opModeIsActive()) {

            vision.update();
            AprilTagDetection tag = vision.getLatestTag();

            if (tag != null) {

                double dist = vision.getDistanceInches(tag);

                if (dist > 65) {
                    drive(0, 0, 0, 0);
                    telemetry.addLine("STOP — Distance > 65 in");
                    telemetry.update();
                    break;
                } else {
                    drive(-DRIVE_FWD, -DRIVE_FWD, -DRIVE_FWD, -DRIVE_FWD);
                }

            } else {
                drive(0, 0, 0, 0);
            }

            sleep(20);
        }

        // SHOOTER ROUTINE
        LSX.setVelocity(SHOOTER_RPM);
        RSX.setVelocity(SHOOTER_RPM);
        sleep(3000);

        IntakeEx.setVelocity(INTAKE_RPM1);
        sleep(1500);
        IntakeEx.setVelocity(0);
        IntakeEx.setVelocity(INTAKE_RPM2);
        sleep(1500);
        IntakeEx.setVelocity(0);

        LSX.setVelocity(0);
        RSX.setVelocity(0);

        stopAll();
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
