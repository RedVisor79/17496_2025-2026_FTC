package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

@Config
@TeleOp(name = "LegoNinjago")
public class LegoNinjago extends LinearOpMode {

    private DcMotor LB; // 0C
    private DcMotor LF; // 1C
    private DcMotor RB; // 2C
    private DcMotor RF; // 3C
    private DcMotorEx LSX; // 0E
    private DcMotorEx RSX; // 1E
    private DcMotorEx IntakeEx; // 2E

    // Drive power values
    double lbPower;
    double lfPower;
    double rbPower;
    double rfPower;

    double velocity=0;

    // Shooter velocity (tunable in FTC Dashboard)
    public static double SHOOTER_VELOCITY1 = 1700;
    public static double SHOOTER_VELOCITY2 = 1400;
    public static double SHOOTER_VELOCITY3 = 1600;
    public static double SHOOTER_VELOCITY4 = 1500;
    public static double INTAKE_VELOCITY = 1000;

    @Override
    public void runOpMode() {
        ElapsedTime runtime = new ElapsedTime();
        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Hardware mapping
        LB = hardwareMap.get(DcMotor.class, "LB");
        LF = hardwareMap.get(DcMotor.class, "LF");
        RB = hardwareMap.get(DcMotor.class, "RB");
        RF = hardwareMap.get(DcMotor.class, "RF");
        LSX = hardwareMap.get(DcMotorEx.class, "LS");
        RSX = hardwareMap.get(DcMotorEx.class, "RS");
        IntakeEx = hardwareMap.get(DcMotorEx.class, "Intake");

        // Motor directions
        LB.setDirection(DcMotor.Direction.FORWARD);
        LF.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.FORWARD);
        RB.setDirection(DcMotor.Direction.FORWARD);
        LSX.setDirection(DcMotorEx.Direction.REVERSE);
        RSX.setDirection(DcMotorEx.Direction.FORWARD);
        IntakeEx.setDirection(DcMotorEx.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            shooterEx();
            intake();
            brakes();

            // Test mode (Y button)
            boolean trial = false;
            if (gamepad1.y) {
                trial = true;
                test(trial);
            }

            // Mecanum drive calculations
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            lbPower = forward - strafe + turn;
            lfPower = forward + strafe + turn;
            rbPower = forward + strafe - turn;
            rfPower = forward - strafe - turn;

            // Normalize
            double max = Math.max(Math.max(Math.abs(lfPower), Math.abs(rfPower)),
                    Math.max(Math.abs(lbPower), Math.abs(rbPower)));

            if (max > 1.0) {
                lbPower /= max;
                lfPower /= max;
                rbPower /= max;
                rfPower /= max;
            }

            // Set drive motor power
            LB.setPower(lbPower);
            LF.setPower(lfPower);
            RB.setPower(rbPower);
            RF.setPower(rfPower);

            // Telemetry (Driver Station + Dashboard)
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Front L/R", "%.2f, %.2f", lfPower, rfPower);
            telemetry.addData("Back L/R", "%.2f, %.2f", lbPower, rbPower);
            telemetry.addData("Shooter Velocity:", velocity);
            telemetry.update();

            dashboard.getTelemetry().addData("Front L/R", "%.2f, %.2f", lfPower, rfPower);
            dashboard.getTelemetry().addData("Back L/R", "%.2f, %.2f", lbPower, rbPower);
            dashboard.getTelemetry().update();
        }
    }

    // Stop all motors if both bumpers pressed
    private void brakes() {
        if (gamepad1.left_bumper && gamepad1.right_bumper) {
            LB.setPower(0);
            LF.setPower(0);
            RB.setPower(0);
            RF.setPower(0);
            LSX.setPower(0);
            RSX.setPower(0);
            IntakeEx.setPower(0);
        }
    }

    // Shooter control (right trigger)
    private void shooterEx() {

        if (gamepad1.dpad_up)
            velocity = SHOOTER_VELOCITY1;
        if (gamepad1.dpad_left)
            velocity = SHOOTER_VELOCITY2;
        if (gamepad1.dpad_right)
            velocity = SHOOTER_VELOCITY3;
        if (gamepad1.dpad_down)
            velocity = SHOOTER_VELOCITY4;

        if (gamepad1.right_trigger > 0) {
            LSX.setVelocity(velocity);
            RSX.setVelocity(velocity);
        } else {
            LSX.setVelocity(0);
            RSX.setVelocity(0);
        }
    }

    // Intake control (left trigger or bumper)
    private void intake() {
        if (gamepad1.left_trigger > 0)
            IntakeEx.setVelocity(INTAKE_VELOCITY);
        else if (gamepad1.left_bumper)
            IntakeEx.setVelocity(-INTAKE_VELOCITY);
        else
            IntakeEx.setVelocity(0);
    }

    // Test mode to individually activate motors
    private void test(boolean trial) {
        while (trial && opModeIsActive()) {
            if (gamepad1.dpad_left) {
                LB.setPower(0.25);
            } else if (gamepad1.dpad_up) {
                LF.setPower(0.25);
            } else if (gamepad1.dpad_right) {
                RB.setPower(0.25);
            } else if (gamepad1.dpad_down) {
                RF.setPower(0.25);
            } else if (gamepad1.y) {
                // Stop test
                LB.setPower(0);
                LF.setPower(0);
                RB.setPower(0);
                RF.setPower(0);
                trial = false;
                break;
            } else {
                LB.setPower(0);
                LF.setPower(0);
                RB.setPower(0);
                RF.setPower(0);
            }
            idle();
        }
    }
}
