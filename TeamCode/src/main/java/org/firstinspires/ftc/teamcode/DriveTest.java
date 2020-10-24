package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.openftc.revextensions2.ExpansionHubMotor;

@TeleOp(group = "ukdrive")
public class DriveTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        ExpansionHubMotor leftFront, leftRear, rightRear, rightFront;
        leftFront = hardwareMap.get(ExpansionHubMotor.class, "FrontLeft");
        leftRear = hardwareMap.get(ExpansionHubMotor.class, "BackLeft");
        rightRear = hardwareMap.get(ExpansionHubMotor.class, "BackRight");
        rightFront = hardwareMap.get(ExpansionHubMotor.class, "FrontRight");


        waitForStart();

        while (opModeIsActive()) {

            boolean slowMode = true;

            double forward = -gamepad1.left_stick_x;
            double right = gamepad1.left_stick_y;
            double spin = gamepad1.right_stick_x;

            leftFront.setPower(forward + right + spin);
            leftRear.setPower(forward + right - spin);
            rightRear.setPower(forward - right + spin);
            rightFront.setPower(forward - right - spin);

            telemetry.update();
        }
    }

}


