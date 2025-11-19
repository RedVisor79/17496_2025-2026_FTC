package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
    public static double SHOOTER_VELOCITY = 1500;
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
            telemetry.addData("Intake:", INTAKE_VELOCITY);
            telemetry.addData("LB:", lbPower);
            telemetry.addData("LF:", lfPower);
            telemetry.addData("RB:", rbPower);
            telemetry.addData("RF:", rfPower);
            telemetry.addData("Left Stick x:", gamepad1.left_stick_y);
            telemetry.addData("Left Stick y:", gamepad1.left_stick_x);
            telemetry.addData("Right Stick x:", gamepad1.right_stick_x);
            telemetry.addData("Right Stick y:", gamepad1.right_stick_y);
            telemetry.update();
        }
    }

    // Shooter control (right trigger)
    private void shooterEx() {

        if (gamepad1.dpad_up&&SHOOTER_VELOCITY<=1800)
            velocity = SHOOTER_VELOCITY+100;
        if (gamepad1.dpad_down&&SHOOTER_VELOCITY>=1400)
            velocity = SHOOTER_VELOCITY-100;

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

}
