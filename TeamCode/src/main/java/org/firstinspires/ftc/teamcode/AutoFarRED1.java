package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name="AutoFarRED1")
public class AutoFarRED1 extends LinearOpMode {

    // Drive motors
    private DcMotor LB, LF, RB, RF;

    // Vision
    private AprilTag vision;

    // Shooter + Intake
    private DcMotorEx LSX, RSX, IntakeEx;

    // Movement constants
    private static final double DRIVE_FWD = 0.5;
    private static final double DRIVE_TURN = 0.5;

    // Shooter constants
    private static final double SHOOTER_RPM = 1702;
    private static final double INTAKE_RPM1 = 1000;

    // Target values
    private static final double TARGET_DIST = 134;  // inches
    private static final double TARGET_X = -12.9;   // raw X meters
    private static final double TOLERANCE = 0.1;

    @Override
    public void runOpMode() throws InterruptedException {

        // Map hardware
        LB = hardwareMap.get(DcMotor.class, "LB");
        LF = hardwareMap.get(DcMotor.class, "LF");
        RB = hardwareMap.get(DcMotor.class, "RB");
        RF = hardwareMap.get(DcMotor.class, "RF");

        LSX = hardwareMap.get(DcMotorEx.class, "LS");
        RSX = hardwareMap.get(DcMotorEx.class, "RS");
        IntakeEx = hardwareMap.get(DcMotorEx.class, "Intake");

        // Set directions
        LB.setDirection(DcMotor.Direction.FORWARD);
        LF.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.FORWARD);
        RF.setDirection(DcMotor.Direction.FORWARD);

        LSX.setDirection(DcMotorEx.Direction.REVERSE);
        RSX.setDirection(DcMotorEx.Direction.FORWARD);
        IntakeEx.setDirection(DcMotorEx.Direction.REVERSE);

        // Enable braking for precise stops
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize vision
        vision = new AprilTag(hardwareMap, telemetry);
        telemetry.addLine("Vision ready");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // ==========================
        // PHASE 1: Drive until distance > TARGET_DIST
        // ==========================
        while (opModeIsActive()) {

            vision.update();
            AprilTagDetection tag = getTag24();  // filter to only tag 24

            if (tag != null) {
                double dist = vision.getDistanceInches(tag);
                telemetry.addData("Distance", dist);

                if (dist >= TARGET_DIST) {
                    stopDrive();
                    telemetry.addLine("Distance reached — stopping");
                    telemetry.update();
                    break;
                } else {
                    drive(DRIVE_FWD, DRIVE_FWD, DRIVE_FWD, DRIVE_FWD);
                }
            } else {
                stopDrive(); // fail-safe
            }

            telemetry.update();
            sleep(20);
        }

        sleep(200);

        // ==========================
        // PHASE 2: Turn until rawX aligned
        // ==========================
        while (opModeIsActive()) {

            vision.update();
            AprilTagDetection tag = getTag24();

            if (tag == null) {
                stopDrive();
                telemetry.addLine("Tag 24 lost — stopping");
                telemetry.update();
                break;
            }

            double rawX = tag.rawPose.x;
            telemetry.addData("rawX", rawX);

            if (Math.abs(rawX - TARGET_X) <= TOLERANCE) {
                stopDrive();
                telemetry.addLine("Aligned with target X");
                telemetry.update();
                break;
            }

            if (rawX > TARGET_X) {
                // Turn left
                drive(-DRIVE_TURN, -DRIVE_TURN, DRIVE_TURN, DRIVE_TURN);
            } else {
                // Turn right
                drive(-DRIVE_TURN, -DRIVE_TURN, DRIVE_TURN, DRIVE_TURN);
            }

            telemetry.update();
            sleep(20);
        }

        // ==========================
        // PHASE 3: Run Shooter + Intake
        // ==========================
        LSX.setVelocity(SHOOTER_RPM);
        RSX.setVelocity(SHOOTER_RPM);
        sleep(3000);

        IntakeEx.setVelocity(INTAKE_RPM1);
        sleep(3000);
        IntakeEx.setVelocity(0);

        LSX.setVelocity(0);
        RSX.setVelocity(0);

        stopAll();
    }

    // Stops drive motors
    private void stopDrive() {
        drive(0,0,0,0);
    }

    private void stopAll() {
        stopDrive();
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

    // Returns only Tag 24, ignoring obelisk
    private AprilTagDetection getTag24() {
        if (vision.getLatestTag() != null && vision.getLatestTag().id == 24) {
            return vision.getLatestTag();
        }
        return null;
    }
}
