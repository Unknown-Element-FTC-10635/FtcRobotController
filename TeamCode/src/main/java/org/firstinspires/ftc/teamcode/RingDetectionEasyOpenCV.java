package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp(name = "Easy Open Cv")
public class RingDetectionEasyOpenCV extends LinearOpMode {

    OpenCvCamera webcam;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(new SamplePipeline());

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });

        double rings = countRing(0.5, 1.5);

        telemetry.addLine("Waiting for start");
        telemetry.addData("Rings: ", rings);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.update();

            if (gamepad1.a) {
                webcam.stopStreaming();
            }

            sleep(100);

        }

    }

    private double countRing(double oneRingThreshold, double fourRingThreshold) {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
        Imgcodecs imageCodecs = new Imgcodecs();
        String importPath = "./ProcessFrame";

        Mat ringFrame = imageCodecs.imread(importPath);

        Scalar mean = Core.mean(ringFrame);
        double val = mean.val[0];

        if (val > fourRingThreshold) {
            return 4;
        } else if (val > oneRingThreshold) {
            return 1;
        } else {
            return 0;
        }

    }

    class SamplePipeline extends OpenCvPipeline {
        boolean viewportPaused;

        @Override
        public Mat processFrame(Mat input) {
            System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
            Imgcodecs imageCodecs = new Imgcodecs();
            String exportPath = "./ProcessFrame";

            Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2BGR);
            Imgproc.cvtColor(input, input, Imgproc.COLOR_BGR2HSV);

            Core.inRange(input, new Scalar(5, 50, 50), new Scalar(15, 255, 255), input);

            if (webcam.getFrameCount() == 1) {
                imageCodecs.imwrite(exportPath, input);
            }

            return input;
        }

        @Override
        public void onViewportTapped() {
            viewportPaused = !viewportPaused;

            if (viewportPaused) {
                webcam.pauseViewport();
            } else {
                webcam.resumeViewport();
            }
        }
    }

}
