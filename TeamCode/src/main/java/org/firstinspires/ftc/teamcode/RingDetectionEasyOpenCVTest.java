package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class RingDetectionEasyOpenCVTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        RingDetectionEasyOpenCV ringDetection = new RingDetectionEasyOpenCV();
        ringDetection.init(hardwareMap);

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double rings = ringDetection.getRingCount();

            telemetry.addData("Rings: ", rings);
            telemetry.addData("Hue Mean: ", ringDetection.getHueMean());
            telemetry.addData("Submat Hue", ringDetection.getInputH());
            telemetry.addData("Submat Saturation ", ringDetection.getInputS());
            telemetry.addData("Submat Value", ringDetection.getInputV());
            telemetry.update();

            ringDetection.reset();

            sleep(100);
        }

        ringDetection.stop();

    }
}
