package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class RingDetectionEasyOpenCVTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        RingDetectionEasyOpenCV ringDetection = new RingDetectionEasyOpenCV(hardwareMap, telemetry);

        telemetry.addLine("Waiting for start");
        telemetry.update();

        ringDetection.start();

        waitForStart();

        while (opModeIsActive() && ringDetection.getFrameCount() < 60) {
            sleep(100);
        }

        telemetry.addData("rings:", ringDetection.getRingCount());
        telemetry.update();

        ringDetection.stop();

    }
}
