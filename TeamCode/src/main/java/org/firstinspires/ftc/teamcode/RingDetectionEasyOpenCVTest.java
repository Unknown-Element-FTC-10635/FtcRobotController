package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class RingDetectionEasyOpenCVTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        RingDetectionCallback callback = new RingDetectionCallback() {
            @Override
            public void ringsCounted(int numberOfRings) {
                telemetry.addData("rings:", numberOfRings);
            }
        };

        RingDetectionEasyOpenCV ringDetection = new RingDetectionEasyOpenCV(hardwareMap, telemetry, callback);

        telemetry.addLine("Waiting for start");
        telemetry.update();

        ringDetection.start();

        waitForStart();

        ringDetection.run();

        telemetry.addLine("Starting webcam");

        while (opModeIsActive()) {
            //telemetry.addData("Hue Mean: ", ringDetection.getHueMean());
            telemetry.update();

            sleep(100);
        }

        ringDetection.stop();

    }
}
