package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ukdrive")
public class EllieTeleOpMode extends OpMode {

    private DcMotor frontLeftDC;
    private DcMotor backLeftDC;
    private DcMotor frontRightDC;
    private DcMotor backRightDC;
    private DcMotor lift;

    private Servo armSwivel;
    private Servo grabber;

    private final int MAX_LIFT_POSITION = 1000;
    private final int MIN_LIFT_POSITION = 0;

    @Override
    public void init() {
        frontLeftDC = hardwareMap.get(DcMotor.class, "Front_Left");
        frontRightDC = hardwareMap.get(DcMotor.class, "Front_Right");
        backLeftDC = hardwareMap.get(DcMotor.class, "Back_Left");
        backRightDC = hardwareMap.get(DcMotor.class, "Back_Right");
        lift = hardwareMap.get(DcMotor.class, "Lift");

        armSwivel = hardwareMap.get(Servo.class, "Arm_Swivel");
        grabber = hardwareMap.get(Servo.class, "Grabber");

        frontLeftDC.setDirection(DcMotor.Direction.FORWARD);
        backLeftDC.setDirection(DcMotor.Direction.FORWARD);
        frontRightDC.setDirection(DcMotor.Direction.REVERSE);
        backRightDC.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Init", "Complete");
        telemetry.update();
    }

    @Override
    public void loop() {
        frontLeftDC.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x) - gamepad1.right_stick_x);
        backLeftDC.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x) - gamepad1.right_stick_x);
        frontRightDC.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x) + gamepad1.right_stick_x);
        backRightDC.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x) + gamepad1.right_stick_x);

        if (lift.getCurrentPosition() > MAX_LIFT_POSITION) {
            lift.setPower(-0.1);
            lift.setTargetPosition(MAX_LIFT_POSITION);
        } else if (lift.getCurrentPosition() < MIN_LIFT_POSITION) {
            lift.setPower(0.1);
            lift.setTargetPosition(MIN_LIFT_POSITION);
        } else {
            lift.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
        }

        if (gamepad1.dpad_left) {
            armSwivel.setPosition(armSwivel.getPosition() - 0.01);
        }
        if (gamepad1.dpad_right) {
            armSwivel.setPosition(armSwivel.getPosition() + 0.01);
        }

        if (gamepad1.dpad_down) {
            grabber.setPosition(0);
        }
        if (gamepad1.dpad_up) {
            grabber.setPosition(1);
        }

        telemetry.addData("Front Left Motor Speed", frontLeftDC.getPower());
        telemetry.addData("Back Left Motor Speed", backLeftDC.getPower());
        telemetry.addData("Front Right Motor Speed", frontRightDC.getPower());
        telemetry.addData("Back Right Motor Speed", backRightDC.getPower());
        telemetry.addData("Arm swivel:", armSwivel.getPosition());
        telemetry.addData("Grabber", grabber.getPosition());
        telemetry.addData("Lift position:", lift.getCurrentPosition());
        telemetry.update();
    }
}
