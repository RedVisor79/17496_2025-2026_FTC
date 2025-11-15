package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

@Config
@Autonomous(name = "Clanker", group = "Linear OpMode")
public class Clanker extends LinearOpMode {

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

    public void runOpMode() throws InterruptedException {
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
        // Telemetry (Driver Station + Dashboard)
        startTelemetryLoop(dashboard, runtime);

        waitForStart();
        if (!isStopRequested()) {
            shooterEx(1);
            sleep(1500);
            intake(1);
            sleep(3000);
            shooterEx(0);
            intake(0);
            move(1, 0, 0);
            sleep(1300);
            move(0,0,0);
        }
    }

    private void move(double forward, double strafe, double turn){
        // Mecanum drive calculations
        //double forward = -gamepad1.left_stick_y;
        //double strafe = gamepad1.left_stick_x;
        //double turn = gamepad1.right_stick_x;

        //forward = -forward;

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

    }

    // Shooter control (right trigger)
    private void shooterEx(int velVal) {

        if (velVal == 1)
            velocity = SHOOTER_VELOCITY1;
        else if (velVal == 2)
            velocity = SHOOTER_VELOCITY2;
        else if (velVal == 3)
            velocity = SHOOTER_VELOCITY3;
        else if (velVal == 4)
            velocity = SHOOTER_VELOCITY4;
        else
            velocity = 0;

        LSX.setVelocity(velocity);
        RSX.setVelocity(velocity);

    }

    // Intake control (left trigger or bumper)
    private void intake(int i) {
        if (i ==1)
            IntakeEx.setVelocity(INTAKE_VELOCITY);
        else if (i ==2 )
            IntakeEx.setVelocity(-INTAKE_VELOCITY);
        else
            IntakeEx.setVelocity(0);
    }

    private void startTelemetryLoop(FtcDashboard dashboard, ElapsedTime runtime) {
        Thread telemetryThread = new Thread(() -> {
            while (!isStopRequested()) {
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Front L/R", "%.2f, %.2f", lfPower, rfPower);
                telemetry.addData("Back L/R", "%.2f, %.2f", lbPower, rbPower);
                telemetry.addData("Shooter Velocity", velocity);
                telemetry.update();

                dashboard.getTelemetry().addData("Front L/R", "%.2f, %.2f", lfPower, rfPower);
                dashboard.getTelemetry().addData("Back L/R", "%.2f, %.2f", lbPower, rbPower);
                dashboard.getTelemetry().addData("Shooter Velocity", velocity);
                dashboard.getTelemetry().update();

                try {
                    Thread.sleep(100); // Update every 100ms
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        });
        telemetryThread.start();
    }
}
