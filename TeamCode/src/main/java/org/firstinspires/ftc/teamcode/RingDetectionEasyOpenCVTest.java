package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class RingDetectionEasyOpenCVTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        RingDetectionEasyOpenCV ringDetection = new RingDetectionEasyOpenCV();
        ringDetection.init(hardwareMap, 2, 7);

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double rings = ringDetection.getRingCount();

            telemetry.addData("Rings: ", rings);
            telemetry.addData("Hue Mean: ", ringDetection.getHueMean());
            telemetry.update();

            ringDetection.reset();

            sleep(100);
        }

        ringDetection.stop();

    }
}
