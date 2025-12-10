package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.teamcode.vision.BallColumnPipeline;
import org.openftc.easyopencv.*;

@Autonomous(name = "Ball Column Detection Only")
public class AlignToBallColumn extends LinearOpMode {

    OpenCvCamera webcam;
    BallColumnPipeline pipeline;

    @Override
    public void runOpMode() throws InterruptedException {

        // ---- Webcam initialization ----
        WebcamName cameraName = hardwareMap.get(WebcamName.class, "WebcamBALLS");

        int cameraMonitorId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(cameraName, cameraMonitorId);

        // Attach pipeline
        pipeline = new BallColumnPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera failed to open!", errorCode);
                telemetry.update();
            }
        });

        // Stream to FTC Dashboard
        CameraStreamServer.getInstance().setSource(webcam);

        telemetry.addLine("Ready. Press start.");
        telemetry.update();
        waitForStart();

        // ---- Main Loop ----
        while (opModeIsActive()) {

            telemetry.addData("Found Column", pipeline.foundColumn);
            telemetry.addData("Column X", pipeline.columnCenterX);
            telemetry.addData("Horizontal Error", pipeline.error);
            telemetry.update();

            // No driving. Only reporting values.
        }
    }
}