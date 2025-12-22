package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
@Autonomous(name = "TurnCalibration")
public class TurnCalibration extends LinearOpMode {

    // Drive motors
    private DcMotor LF, LB, RF, RB;

    // Vision
    private AprilTag vision;

    // ==============================
    // TUNING CONSTANTS
    // ==============================
    private static final double TURN_POWER = 0.3;

    // CALIBRATE THIS:
    // degrees per second when turning at 0.3 power
    public static double DEG_PER_SEC_AT_0_3 = 150.0;

    private static final double TARGET_ANGLE_DEG = 45.0;
    private static final double TAG_WAIT_TIMEOUT = 2.0;

    @Override
    public void runOpMode() throws InterruptedException {

        // ==============================
        // HARDWARE MAP
        // ==============================
        LF = hardwareMap.get(DcMotor.class, "LF");
        LB = hardwareMap.get(DcMotor.class, "LB");
        RF = hardwareMap.get(DcMotor.class, "RF");
        RB = hardwareMap.get(DcMotor.class, "RB");

        LF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.FORWARD);
        RF.setDirection(DcMotor.Direction.FORWARD);
        RB.setDirection(DcMotor.Direction.FORWARD);

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Initializing AprilTag camera...");
        telemetry.update();

        vision = new AprilTag(hardwareMap, telemetry);

        telemetry.addLine("Ready. Press PLAY.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // ==============================
        // PHASE 1: WAIT FOR STABLE TAG
        // ==============================
        double savedThetaAbsDeg = 0.0;
        boolean gotAngle = false;

        double startTime = getRuntime();

        while (opModeIsActive()) {
            vision.update();
            AprilTagDetection tag = vision.getTagById(24);

            if (tag != null && vision.isStable()) {
                double theta = vision.getAngleDegrees(tag);
                savedThetaAbsDeg = Math.abs(theta);
                gotAngle = true;

                telemetry.addData("Saved theta (abs deg)", "%.2f", savedThetaAbsDeg);
                telemetry.update();
                break;
            }

            if (getRuntime() - startTime > TAG_WAIT_TIMEOUT) {
                telemetry.addLine("Tag timeout — using 0°");
                telemetry.update();
                break;
            }

            sleep(30);
        }

        // ==============================
        // PHASE 2: COMPUTE TURN
        // ==============================
        double turnDegrees = TARGET_ANGLE_DEG - savedThetaAbsDeg;
        turnDegrees = Math.max(0.0, turnDegrees); // force positive

        double turnTimeSec = turnDegrees / DEG_PER_SEC_AT_0_3;
        long turnTimeMs = (long)(turnTimeSec * 1000);

        telemetry.addData("Turn degrees", "%.2f", turnDegrees);
        telemetry.addData("Turn time (ms)", turnTimeMs);
        telemetry.update();

        sleep(300);

        // ==============================
        // PHASE 3: BLIND LEFT TURN
        // ==============================
        // Left turn: left motors backward, right motors forward
        drive(-TURN_POWER, -TURN_POWER, +TURN_POWER, +TURN_POWER);
        sleep(turnTimeMs);
        drive(0, 0, 0, 0);

        telemetry.addLine("Turn complete");
        telemetry.update();

        sleep(1000);
    }

    // ==============================
    // DRIVE HELPER
    // ==============================
    private void drive(double lf, double lb, double rf, double rb) {
        LF.setPower(lf);
        LB.setPower(lb);
        RF.setPower(rf);
        RB.setPower(rb);
    }
}
