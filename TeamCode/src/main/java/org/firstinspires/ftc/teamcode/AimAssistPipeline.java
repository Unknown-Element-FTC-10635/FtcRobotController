package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class AimAssistPipeline {
    private OpenCvCamera webcam;
    private AimAssistProcessing pipeline;

    private final HardwareMap hardwareMap;

    public AimAssistPipeline(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void start() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);

        pipeline = new AimAssistProcessing();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }
        });

    }

    public Point getCenterOfHighGoal() {
        return pipeline.getCenterOfHighGoal();
    }


    class AimAssistProcessing extends OpenCvPipeline {
        Point centerOfHighGoal;

        @Override
        public Mat processFrame(Mat input) {
            FindHighGoal highGoal = new FindHighGoal();
            centerOfHighGoal = highGoal.findHighGoal(input);

            return input;
        }

        public Point getCenterOfHighGoal() {
            return centerOfHighGoal;
        }
    }
}
