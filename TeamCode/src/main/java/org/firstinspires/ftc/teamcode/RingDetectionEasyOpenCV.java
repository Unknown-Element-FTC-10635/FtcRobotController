package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class RingDetectionEasyOpenCV {

    private OpenCvCamera webcam;
    private RingDetectionEasyOpenCV.CameraPipeline pipeline;

    private final HardwareMap hardwareMap;
    private final double oneRingThreshold;
    private final double fourRingThreshold;

    public RingDetectionEasyOpenCV(HardwareMap hardwareMap, double oneRingThreshold, double fourRingThreshold) {
        this.hardwareMap = hardwareMap;
        this.oneRingThreshold = oneRingThreshold;
        this.fourRingThreshold = fourRingThreshold;
    }

    public void start(RingDetectionCallback callback) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new CameraPipeline(oneRingThreshold, fourRingThreshold, callback);
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });
    }

    public int getRingCount() {
        return pipeline.getRingCount();
    }

    public double getHueMean() {
        return pipeline.getHueMean();
    }

    public int getFrame() {
        return webcam.getFrameCount();
    }

    public void reset() {
        pipeline.reset();
    }

    public void stop() {
        webcam.closeCameraDevice();
    }

    class CameraPipeline extends OpenCvPipeline {
        private double oneRingThreshold;
        private double fourRingThreshold;
        private RingDetectionCallback callback;
        private double hueMean = 0;
        private int frames = 0;

        public CameraPipeline(double oneRingThreshold, double fourRingThreshold, RingDetectionCallback callback) {
            this.oneRingThreshold = oneRingThreshold;
            this.fourRingThreshold = fourRingThreshold;
            this.callback = callback;
        }

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);

            Core.inRange(input, new Scalar(5, 50, 50), new Scalar(22, 255, 255), input);

            hueMean = hueMean + processRing(input);
            frames++;

            if (frames == 30) {
                switch (getRingCount()) {
                    case 0:
                        callback.noRings();
                        break;
                    case 1:
                        callback.oneRing();
                        break;
                    case 4:
                        callback.fourRings();
                        break;
                    default:
                        throw new IllegalStateException("Unknown ring count");
                }
            }

            return input;
        }

        private double processRing(Mat frame) {
            Scalar mean = Core.mean(frame);
            return mean.val[0];
        }

        public int getRingCount() {
            double val = getHueMean();

            if (val > fourRingThreshold) {
                return 4;
            } else if (val > oneRingThreshold) {
                return 1;
            } else {
                return 0;
            }

        }

        public void reset() {
            hueMean = 0;
            frames = 0;
        }

        public double getHueMean() {
            return hueMean / frames;
        }
    }
}
