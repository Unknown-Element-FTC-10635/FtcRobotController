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
            public void oneRing() {
                telemetry.addData("rings:", 1);
            }

            @Override
            public void noRings() {
                telemetry.addData("rings:", 0);
            }

            @Override
            public void fourRings() {
                telemetry.addData("rings:", 4);
            }
        });

        while (opModeIsActive()) {
            telemetry.addData("Hue Mean: ", ringDetection.getHueMean());
            telemetry.addData("Frames: ", ringDetection.getFrame());
            telemetry.update();

            //ringDetection.reset();

            sleep(100);
        }

        //ringDetection.stop();

    }
}
