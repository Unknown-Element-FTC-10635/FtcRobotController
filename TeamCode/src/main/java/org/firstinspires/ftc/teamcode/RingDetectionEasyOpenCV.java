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
    private final double ONE_RING_THRESHOLD = 5.0;
    private final double FOUR_RING_THRESHOLD = 7.0;

    private OpenCvCamera webcam;
    private RingDetectionEasyOpenCV.CameraPipeline pipeline;

    int cameraMonitorViewId;

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final RingDetectionCallback callback;

    public RingDetectionEasyOpenCV(HardwareMap hardwareMap, Telemetry telemetry, RingDetectionCallback callback) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.callback = callback;
    }

    public void start() {
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new CameraPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });
    }

    public void run() {
        while (pipeline.getFrames() < 60) {
            // Do literally nothing
        }
        callback.ringsCounted(pipeline.getRingCount());
        //webcam.stopStreaming();
    }

    public int getRingCount() {
        return pipeline.getRingCount();
    }

    public double getHueMean() {
        return pipeline.getHueMean();
    }

    public int getFrame() {
        return pipeline.getFrames();
    }

    public void stop() {
        webcam.stopStreaming();
        //webcam.closeCameraDevice();
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
