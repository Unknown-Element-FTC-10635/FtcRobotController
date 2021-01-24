package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class RingDetectionEasyOpenCV {
    private final double ONE_RING_THRESHOLD = 1.5;
    private final double FOUR_RING_THRESHOLD = 3.5;

    private OpenCvCamera webcam;
    private RingDetectionEasyOpenCV.CameraPipeline pipeline;

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    public RingDetectionEasyOpenCV(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    public void start() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new CameraPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                telemetry.addLine("Starting camera stream");
                telemetry.update();
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }
        });
    }

    public int getFrameCount() {
        return pipeline.getFrames();
    }

    public int getRingCount() {
        return pipeline.getRingCount();
    }

    public void stop() {
        webcam.stopStreaming();
    }

    class CameraPipeline extends OpenCvPipeline {
        private double hueMean = 0;
        private int frames = 0;

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);
            Imgproc.GaussianBlur(input, input, new Size(5, 5), 0);

            Core.inRange(input, new Scalar(5, 50, 50), new Scalar(22, 255, 255), input);

            hueMean = processRing(input);

            frames++;

            return input;
        }

        private double processRing(Mat frame) {
            Scalar mean = Core.mean(frame);
            return mean.val[0];
        }

        public int getRingCount() {
            double val = getHueMean();
            telemetry.addData("hueMean: ", val);
            if (val > FOUR_RING_THRESHOLD) {
                telemetry.addData("rings: ", 4);
                return 4;
            } else if (val > ONE_RING_THRESHOLD) {
                telemetry.addData("rings: ", 1);
                return 1;
            } else {
                telemetry.addData("rings: ", 0);
                return 0;
            }

        }

        public double getHueMean() {
            return hueMean;
        }

        public int getFrames() {
            return frames;
        }
    }
}
