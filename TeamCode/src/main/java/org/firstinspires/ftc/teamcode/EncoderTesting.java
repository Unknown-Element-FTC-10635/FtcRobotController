package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class EncoderTesting extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        DcMotor wobbleArm = hardwareMap.get(DcMotor.class, "wobble");

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int rotateToOpened = (int)(180 * 7);

        waitForStart();

        wobbleArm.setPower(.3);
        wobbleArm.setTargetPosition(800);
        wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (wobbleArm.isBusy()) {}

        wobbleArm.setTargetPosition(0);

        while (wobbleArm.isBusy()) {}

        wobbleArm.setTargetPosition(800);

        while (wobbleArm.isBusy()) {}

        wobbleArm.setTargetPosition(0);

        while (opModeIsActive()) {
            telemetry.addData("Arm", wobbleArm.getCurrentPosition());
            telemetry.update();
            sleep(250);
        }


    }

}
