package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
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

        CameraPipeline pipeline = new CameraPipeline(1, 4);
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double rings = pipeline.getRingCount();

            telemetry.addData("Rings: ", rings);
            telemetry.addData("Hue Mean: ", pipeline.getHueMean());
            telemetry.addData("Submat Mean", pipeline.getInputMean());
            telemetry.update();

            pipeline.reset();

            if (gamepad1.a) {
                webcam.stopStreaming();
            }

            sleep(100);

        }

    }

    class CameraPipeline extends OpenCvPipeline {
        private double oneRingThreshold = 0.5;
        private double fourRingThreshold = 1.5;
        private double hueMean = 0;
        private int frames = 0;
        private Scalar inputMean;
        Rect target = new Rect(new Point(430, 0), new Point(480, 50));

        public CameraPipeline() {
        }

        public CameraPipeline(double oneRingThreshold, double fourRingThreshold) {
            this.oneRingThreshold = oneRingThreshold;
            this.fourRingThreshold = fourRingThreshold;
        }

        @Override
        public Mat processFrame(Mat input) {
            Mat mask = new Mat();
            Imgproc.cvtColor(input, mask, Imgproc.COLOR_RGB2HSV);

            if (webcam.getFrameCount() == 10) {
                Mat submat = input.submat(target);

                inputMean = Core.mean(submat);
            }

            Core.inRange(mask, new Scalar(5, 50, 50), new Scalar(22, 255, 255), mask);

            hueMean = hueMean + processRing(mask);
            frames++;

            return mask;
        }

        private double processRing(Mat frame) {
            Scalar mean = Core.mean(frame);
            return mean.val[0];
        }

        public int getRingCount() {
            double val = hueMean / frames;

            if (val > fourRingThreshold) {
                return 4;
            } else if (val > oneRingThreshold) {
                return 1;
            } else {
                return 0;
            }

        }

        public double getHueMean() {
            return hueMean / frames;
        }

        public void reset() {
            hueMean = 0;
            frames = 0;
        }

        public Scalar getInputMean() {
            return inputMean;
        }
    }

}
