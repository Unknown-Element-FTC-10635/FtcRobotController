package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class RingDetectionEasyOpenCVTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        RingDetectionEasyOpenCV ringDetection = new RingDetectionEasyOpenCV(hardwareMap, 3, 5);

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        telemetry.addLine("Starting webcam");
        ringDetection.start(new RingDetectionCallback() {
            @Override
            public void ringsCounted(int numberOfRings) throws InterruptedException {
                telemetry.addData("rings:", numberOfRings);
            }
        });

        while (opModeIsActive()) {
            telemetry.addData("Hue Mean: ", ringDetection.getHueMean());
            telemetry.addData("Frames: ", ringDetection.getFrame());
            telemetry.addData("ID: ", ringDetection.deviceID());
            telemetry.update();

            //ringDetection.reset();

            sleep(100);
        }

        //ringDetection.stop();

    }
}
