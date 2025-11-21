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
    boolean shooting=false;

    // Shooter velocity (tunable in FTC Dashboard)
    public static double SHOOTER_VELOCITY = 0;
    public static double INTAKE_VELOCITY = 1600;

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

            lbPower = forward - 0.65*strafe + turn;
            lfPower = forward + strafe + turn;
            rbPower = forward + 0.65*strafe - turn;
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
            telemetry.addData("Robot Values:","");
            telemetry.addData("Shooter Velocity:", SHOOTER_VELOCITY);
            telemetry.addData("Intake:", INTAKE_VELOCITY);
            telemetry.addData("LB:", lbPower);
            telemetry.addData("LF:", lfPower);
            telemetry.addData("RB:", rbPower);
            telemetry.addData("RF:", rfPower);
            telemetry.addData("Input Data:","");
            telemetry.addData("Left Stick x:", gamepad1.left_stick_y);
            telemetry.addData("Left Stick y:", gamepad1.left_stick_x);
            telemetry.addData("Right Stick x:", gamepad1.right_stick_x);
            telemetry.addData("Right Stick y:", gamepad1.right_stick_y);
            telemetry.update();
        }
    }

    // Shooter control
    private void shooterEx() {

        if (gamepad1.dpad_up)
            SHOOTER_VELOCITY=1750;
        if (gamepad1.dpad_left)
            SHOOTER_VELOCITY=1425;

        if (gamepad1.a){
            LSX.setVelocity(SHOOTER_VELOCITY);
            RSX.setVelocity(SHOOTER_VELOCITY);
            shooting=true;
        }
        if (gamepad1.x){
            LSX.setVelocity(0);
            RSX.setVelocity(0);
            shooting = false;
        }
        if (gamepad1.b&&!shooting){
            LSX.setVelocity(-SHOOTER_VELOCITY);
            RSX.setVelocity(-SHOOTER_VELOCITY);
            shooting=true;
            sleep(10);
            LSX.setVelocity(0);
            RSX.setVelocity(0);
            shooting = false;
        }
    }

    // Intake control
    private void intake() {
        if (gamepad1.right_trigger>0)
            IntakeEx.setVelocity(INTAKE_VELOCITY);
        else if (gamepad1.left_trigger>0)
            IntakeEx.setVelocity(-INTAKE_VELOCITY);
        else
            IntakeEx.setVelocity(0);
    }

}
