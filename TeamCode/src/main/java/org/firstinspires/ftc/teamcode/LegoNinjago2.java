package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

@Config
@TeleOp(name = "LegoNinjago2", group = "Calibration")
public class LegoNinjago2 extends LinearOpMode {

    private DcMotor LB, LF, RB, RF;

    // Fixed strafe offsets
    public static double LB_OFFSET = 1.050;
    public static double LF_OFFSET = 1.027;
    public static double RB_OFFSET = 1.037;
    public static double RF_OFFSET = 1.000;

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Hardware mapping
        LB = hardwareMap.get(DcMotor.class, "LB");
        LF = hardwareMap.get(DcMotor.class, "LF");
        RB = hardwareMap.get(DcMotor.class, "RB");
        RF = hardwareMap.get(DcMotor.class, "RF");

        // Motor directions
        LB.setDirection(DcMotor.Direction.FORWARD);
        LF.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.FORWARD);
        RB.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addLine("LegoNinjago2 Ready");
        telemetry.addLine("Strafe LEFT  : dpad_left");
        telemetry.addLine("Strafe RIGHT : dpad_right");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            double forward = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            double strafe = 0;
            if (gamepad1.dpad_left)  strafe = -1;
            if (gamepad1.dpad_right) strafe = 1;

            double lbPower = forward - LB_OFFSET * strafe + turn;
            double lfPower = forward + LF_OFFSET * strafe + turn;
            double rbPower = forward + RB_OFFSET * strafe - turn;
            double rfPower = forward - RF_OFFSET * strafe - turn;

            // Normalize
            double max = Math.max(
                    Math.max(Math.abs(lbPower), Math.abs(lfPower)),
                    Math.max(Math.abs(rbPower), Math.abs(rfPower))
            );
            if (max > 1.0) {
                lbPower /= max;
                lfPower /= max;
                rbPower /= max;
                rfPower /= max;
            }

            LB.setPower(lbPower);
            LF.setPower(lfPower);
            RB.setPower(rbPower);
            RF.setPower(rfPower);

            telemetry.addData("LB", "%.2f", lbPower);
            telemetry.addData("LF", "%.2f", lfPower);
            telemetry.addData("RB", "%.2f", rbPower);
            telemetry.addData("RF", "%.2f", rfPower);

            telemetry.addLine();
            telemetry.addData("LB_OFFSET", LB_OFFSET);
            telemetry.addData("LF_OFFSET", LF_OFFSET);
            telemetry.addData("RB_OFFSET", RB_OFFSET);
            telemetry.addData("RF_OFFSET", RF_OFFSET);

            telemetry.update();
        }
    }
}
