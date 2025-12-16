package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name = "BALL_COLLECTION")
public class BallCollection extends LinearOpMode {

    // Drivetrain
    private DcMotor LF, LB, RF, RB;

    // Intake + Launcher
    private DcMotorEx IntakeEx, LSX, RSX;

    // Vision
    private AprilTag vision;

    // Constants
    private static final double TURN_POWER = 0.3;
    private static final double TARGET_ANGLE_DEG = 45.0;

    private static final double LAUNCHER_RPM = 1700;

    @Override
    public void runOpMode() throws InterruptedException {

        // ---------------------------
        // Hardware Init
        // ---------------------------
        LF = hardwareMap.get(DcMotor.class, "LF");
        LB = hardwareMap.get(DcMotor.class, "LB");
        RF = hardwareMap.get(DcMotor.class, "RF");
        RB = hardwareMap.get(DcMotor.class, "RB");

        IntakeEx = hardwareMap.get(DcMotorEx.class, "Intake");
        LSX = hardwareMap.get(DcMotorEx.class, "LS");
        RSX = hardwareMap.get(DcMotorEx.class, "RS");

        LF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.FORWARD);
        RF.setDirection(DcMotor.Direction.FORWARD);
        RB.setDirection(DcMotor.Direction.FORWARD);

        IntakeEx.setDirection(DcMotorEx.Direction.REVERSE);
        LSX.setDirection(DcMotorEx.Direction.REVERSE);
        RSX.setDirection(DcMotorEx.Direction.FORWARD);

        vision = new AprilTag(hardwareMap, telemetry);

        waitForStart();
        if (isStopRequested()) return;

        // ---------------------------
        // STEP 1 — START INTAKE
        // ---------------------------
        IntakeEx.setPower(1.0);

        // ---------------------------
        // STEP 2 — DRIVE FORWARD
        // ---------------------------
        drive(0.4, 0.4, 0.4, 0.4);
        sleep(1200);
        stopDrive();

        // ---------------------------
        // STEP 3 — INTAKE OFF
        // ---------------------------
        IntakeEx.setPower(0);

        // ---------------------------
        // STEP 4 — DRIVE BACKWARD
        // ---------------------------
        drive(-0.6, -0.6, -0.6, -0.6);
        sleep(1200);
        stopDrive();

        // ---------------------------
// STEP 5A — SEARCH FOR APRILTAG 24
// ---------------------------
        long searchStartTime = System.currentTimeMillis();
        boolean tagFound = false;

        while (opModeIsActive() && !tagFound) {
            vision.update();
            AprilTagDetection tag = vision.getTagById(24);

            if (tag != null && tag.rawPose != null) {
                tagFound = true;
                break;
            }

            // Slow CCW search turn
            drive(-0.2, -0.2, 0.2, 0.2);

            // Safety timeout (2 seconds)
            if (System.currentTimeMillis() - searchStartTime > 2000) {
                break;
            }
        }

        stopDrive();

        if (!tagFound) {
            telemetry.addLine("Tag 24 NOT FOUND");
            telemetry.update();
            return; // or continue without shooting if you prefer
        }

        // ---------------------------
        // STEP 5B — TURN CCW TO 45° USING TAG
        // ---------------------------
        long turnStartTime = System.currentTimeMillis();

        while (opModeIsActive()) {
            vision.update();
            AprilTagDetection tag = vision.getTagById(24);

            if (tag != null && tag.rawPose != null) {
                double rawX = tag.rawPose.x;
                double rawZ = tag.rawPose.z;

                double angleDeg = Math.toDegrees(Math.atan2(rawX, rawZ));

                telemetry.addData("Angle", angleDeg);
                telemetry.update();

                if (Math.abs(angleDeg) >= TARGET_ANGLE_DEG) {
                    break;
                }
            }

            // CCW turn
            drive(-TURN_POWER, -TURN_POWER, TURN_POWER, TURN_POWER);
        }

        stopDrive();
        long turnDuration = System.currentTimeMillis() - turnStartTime;


        // ---------------------------
        // STEP 6 — SLIGHT OUTTAKE
        // ---------------------------
        IntakeEx.setPower(-0.2);
        sleep(300);
        IntakeEx.setPower(0);

        // ---------------------------
        // STEP 7 — SPIN UP LAUNCHERS
        // ---------------------------
        LSX.setVelocity(LAUNCHER_RPM);
        RSX.setVelocity(LAUNCHER_RPM);
        sleep(2000);

        // ---------------------------
        // STEP 8 — FEED BALLS
        // ---------------------------
        IntakeEx.setPower(1.0);
        sleep(1500);
        IntakeEx.setPower(0);

        LSX.setVelocity(0);
        RSX.setVelocity(0);

        // ---------------------------
        // STEP 9 — TURN BACK CW (SAME AMOUNT)
        // ---------------------------
        drive(TURN_POWER, TURN_POWER, -TURN_POWER, -TURN_POWER);
        sleep(turnDuration);
        stopDrive();
    }

    // ================= Helper Methods =================

    private void drive(double lf, double lb, double rf, double rb) {
        LF.setPower(lf);
        LB.setPower(lb);
        RF.setPower(rf);
        RB.setPower(rb);
    }

    private void stopDrive() {
        LF.setPower(0);
        LB.setPower(0);
        RF.setPower(0);
        RB.setPower(0);
    }
}
