package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "LegoNinjago")
public class LegoNinjago extends LinearOpMode{

    //motors
    private DcMotor LB; //0D
    private DcMotor LF; //1D
    private DcMotor RB; //2D
    private DcMotor RF; //3D
    private DcMotor LS; //0E
    private DcMotor RS; //1E
    private DcMotor Intake;//2E

    double lbPower;
    double lfPower;
    double rbPower;
    double rfPower;

    public void runOpMode() {
        ElapsedTime runtime = new ElapsedTime();

        LB = hardwareMap.get(DcMotor.class, "LB");
        LF = hardwareMap.get(DcMotor.class, "LF");
        RB = hardwareMap.get(DcMotor.class, "RB");
        RF = hardwareMap.get(DcMotor.class, "RF");
        LS = hardwareMap.get(DcMotor.class, "LS");
        RS = hardwareMap.get(DcMotor.class, "RS");
        Intake = hardwareMap.get(DcMotor.class, "Intake");

        LB.setDirection(DcMotor.Direction.FORWARD);
        LF.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.FORWARD);
        RF.setDirection(DcMotor.Direction.FORWARD);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        //actual code lol
        while (opModeIsActive()) {
            shooter();//it shoots
            intake();//it intakes
            //drive();//it drives
            double max;
            double forward = -1*gamepad1.left_stick_y;
            double strafe = -1*gamepad1.left_stick_x;
            double direction = gamepad1.right_stick_x;

            lbPower = forward-strafe+direction;
            lfPower = forward+strafe+direction;
            rbPower = forward+strafe-direction;
            rfPower = forward-strafe-direction;

            max = Math.max(Math.abs(lfPower), Math.abs(rfPower));
            max = Math.max(max, Math.abs(lbPower));
            max = Math.max(max, Math.abs(rbPower));

            if (max > 1.0) {
                lbPower  /= max;
                lfPower /= max;
                rbPower   /= max;
                rfPower  /= max;
            }

            LB.setPower(lbPower);
            LF.setPower(lfPower);
            RB.setPower(rbPower);
            RF.setPower(rfPower);

            telemetry.addData("Status", "Run Time: "+runtime.toString());
            telemetry.addData("Front left/Right", JavaUtil.formatNumber(lfPower, 4, 2)+", "+JavaUtil.formatNumber(rfPower, 4, 2));
            telemetry.addData("Back left/Right", JavaUtil.formatNumber(lbPower, 4, 2)+", "+JavaUtil.formatNumber(rbPower, 4, 2));
            telemetry.update();
        }
    }

    private void shooter(){//shooter
        LS.setPower(-gamepad1.right_trigger);
        RS.setPower(gamepad1.right_trigger);
    }
    private void intake(){
        if (gamepad1.dpad_left)
            Intake.setPower(1);
        else if (gamepad1.dpad_right)
            Intake.setPower(-1);
        else
            Intake.setPower(0);
    }

    private void test(){
        boolean trial = false;
        if (gamepad1.y) {
            trial = !trial;
        }
        while (trial) {
            if (gamepad1.dpad_left) {
                LB.setPower(0.25);
                LF.setPower(0);
                RB.setPower(0);
                RF.setPower(0);
            }
            else if (gamepad1.dpad_up) {
                LB.setPower(0);
                LF.setPower(0.25);
                RB.setPower(0);
                RF.setPower(0);
            }
            else if (gamepad1.dpad_right) {
                LB.setPower(0);
                LF.setPower(0);
                RB.setPower(0.25);
                RF.setPower(0);
            }
            else if (gamepad1.dpad_down) {
                LB.setPower(0);
                LF.setPower(0);
                RB.setPower(0);
                RF.setPower(0.25);
            }
            else if (gamepad1.y) {
                LB.setPower(0);
                LF.setPower(0);
                RB.setPower(0);
                RF.setPower(0);
                trial = false;
            }
        }
    }
}