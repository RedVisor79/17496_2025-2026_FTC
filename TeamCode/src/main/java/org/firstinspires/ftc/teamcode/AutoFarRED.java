package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name="AutoFarRED")
public class AutoFarRED extends LinearOpMode {

    // Drive motors
    private DcMotor LB, LF, RB, RF;

    // Vision
    private AprilTag vision;

    // Shooter + Intake
    private DcMotorEx LSX, RSX, IntakeEx;

    // Constants
    private static final double DRIVE_FWD = 0.5;
    private static final double DRIVE_TURN = 0.5;
    private static final double SHOOTER_RPM = 1702;
    private static final double INTAKE_RPM1 = 1000;
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

        telemetry.addLine("Initializing AprilTag Vision...");
        telemetry.update();

        // Create your AprilTag vision object
        vision = new AprilTag(hardwareMap, telemetry);
        telemetry.addLine("Vision Ready!");
        telemetry.addLine("Press PLAY to begin testing");
        telemetry.update();

        // Target Values
        double targetDist = 134, targetX = -12.9, tolerance = 0.1;

        waitForStart();
        if (isStopRequested()) return;

        // Drive until targetDist is reached
        while (opModeIsActive()) {

            // Update AprilTag readings
            vision.update();

            AprilTagDetection tag = vision.getLatestTag();

            // Drive Fwd until...
            if (tag != null) {

                double dist = vision.getDistanceInches(tag);
                telemetry.addData("Distance (in)", dist);

                if (dist > targetDist) {

                    // Stop driving
                    drive(0, 0, 0, 0);

                    telemetry.addLine("STOP — Distance > /*VALUE*/ in");
                    telemetry.update();

                    break;
                }
                else {
                    drive(DRIVE_FWD, DRIVE_FWD, DRIVE_FWD, DRIVE_FWD);
                }
            }
            else {
                // Lost tag — stop driving to be safe
                drive(0, 0, 0, 0);
            }
            sleep(50);

            // Turn right until...
            if (tag != null) {

                double rawX = tag.rawPose.x;
                telemetry.addData("rawX", rawX);

                // Stop turning when rawX is within target tolerance
                if (Math.abs(rawX - targetX) <= tolerance) {
                    drive(0, 0, 0, 0);
                    telemetry.addLine("STOP — rawX reached target range");
                    telemetry.update();
                    break;
                }

                // If rawX is above target, turn LEFT
                if (rawX > targetX) {
                    drive(-DRIVE_TURN, -DRIVE_TURN, DRIVE_TURN, DRIVE_TURN);
                    telemetry.addLine("Turning LEFT");
                }
                // If rawX is below target, turn RIGHT
                else if (rawX < targetX) {
                    drive(DRIVE_TURN, DRIVE_TURN, -DRIVE_TURN, -DRIVE_TURN);
                    telemetry.addLine("Turning RIGHT");
                }

                telemetry.update();
            }
            else {
                // Fail-safe: no tag → stop
                drive(0, 0, 0, 0);
            }


            // Telemetry
            vision.addTelemetry();
            vision.addDashboardTelemetry(tag);
            telemetry.update();

            sleep(20);
        }

        // Spin SHOOTER UP (1400 RPM) for 1 sec
        LSX.setVelocity(SHOOTER_RPM);
        RSX.setVelocity(SHOOTER_RPM);
        sleep(3000);

        // Run INTAKE for 3 sec WHILE shooter stays running
        IntakeEx.setVelocity(INTAKE_RPM1);
        sleep(3000);
        // Stop intake
        IntakeEx.setVelocity(0);

        // Stop Shooter
        LSX.setVelocity(0);
        RSX.setVelocity(0);

        // STOP EVERYTHING
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
