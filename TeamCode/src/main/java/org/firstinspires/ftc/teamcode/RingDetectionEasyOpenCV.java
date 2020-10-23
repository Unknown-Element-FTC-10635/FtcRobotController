package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

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

public class RingDetectionEasyOpenCV {

    private OpenCvCamera webcam;
    private RingDetectionEasyOpenCV.CameraPipeline pipeline;

    public void init(HardwareMap hardwareMap, double oneRingThreshold, double fourRingThreshold) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new CameraPipeline(oneRingThreshold, fourRingThreshold);
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });
    }

    public double getRingCount() {
        return pipeline.getRingCount();
    }

    public double getHueMean() {
        return pipeline.getHueMean();
    }

    public int getFrame() {
        return webcam.getFrameCount();
    }

    public double getInputH() {
        return pipeline.getInputH();
    }

    public double getInputS() {
        return pipeline.getInputS();
    }

    public double getInpusV() {
        return pipeline.getInputV();
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
        private double hueMean = 0;
        private int frames = 0;

//        private double BASE_GREY_HUE = 169.1;
//        private double BASE_GREY_SATURATION = 154;
//        private double BASE_GREY_VALUE = 134.6;

        private double BASE_GREY_HUE = 144.67;
        private double BASE_GREY_SATURATION = 115.52;
        private double BASE_GREY_VALUE = 129.42;

        private Scalar lowerHSV = new Scalar(5, 50, 50);
        private Scalar upperHSV = new Scalar(22, 255, 255);

        private double inputH = 0;
        private double inputS = 0;
        private double inputV = 0;


        public CameraPipeline(double oneRingThreshold, double fourRingThreshold) {
            this.oneRingThreshold = oneRingThreshold;
            this.fourRingThreshold = fourRingThreshold;
        }

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);

            if (webcam.getFrameCount() >= 10) {
                adjustBrightness(input);
                //adjustForBrightness(input);
            }

            Core.inRange(input, lowerHSV, upperHSV, input);

            hueMean = hueMean + processRing(input);
            frames++;

            return input;
        }

        private double processRing(Mat frame) {
            Scalar mean = Core.mean(frame);
            return mean.val[0];
        }

        private void adjustBrightness(Mat input) {
            Mat submat = input.submat(new Rect(new Point(430, 0), new Point(480, 50)));
            Scalar inputMean = Core.mean(submat);

            double deltaV = BASE_GREY_VALUE - inputMean.val[2];

            double alpha = 1.0;
            input.convertTo(input, -1, alpha, deltaV);
        }

        private void adjustForBrightness(Mat frame) {
            Mat submat = frame.submat(new Rect(new Point(430, 0), new Point(480, 50)));
            Scalar inputMean = Core.mean(submat);

            inputH = inputMean.val[0];
            inputS = inputMean.val[1];
            inputV = inputMean.val[2];

            double deltaH = BASE_GREY_HUE - inputH;
            double deltaS = BASE_GREY_SATURATION - inputS;
            double deltaV= BASE_GREY_VALUE - inputV;

            lowerHSV = new Scalar(
                    5 + (deltaH),
                    50 + (deltaS),
                    50 + (deltaV));

            upperHSV = new Scalar(
                    22 + (deltaH),
                    255,
                    255);
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

        public double getInputH() {
            return inputH;
        }

        public double getInputS() {
            return inputS;
        }

        public double getInputV() {
            return inputV;
        }
    }
}
