package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Point;

@TeleOp(name = "ukdrive")
public class EllieTeleOpMode extends OpMode {

    private DcMotor frontLeftDC;
    private DcMotor backLeftDC;
    private DcMotor frontRightDC;
    private DcMotor backRightDC;
    private DcMotor lift;
    private DcMotor arm;

    private Servo armSwivel;
    private Servo grabber;
    private Servo ringGrabber;

    private boolean ringGrabberOpen;

    private double wheelMultiplier;

    private AimAssistPipeline aimAssist;

    private ElapsedTime r3Timer = new ElapsedTime();
    private ElapsedTime aTimer = new ElapsedTime();

    private final int MAX_LIFT_POSITION = 1000;
    private final int MIN_LIFT_POSITION = 0;

    private final Point RING_LAUNCH_POINT = new Point(320, 240);

    @Override
    public void init() {
        aimAssist = new AimAssistPipeline(hardwareMap);
        aimAssist.start();

        frontLeftDC = hardwareMap.get(DcMotor.class, "Front_Left");
        frontRightDC = hardwareMap.get(DcMotor.class, "Front_Right");
        backLeftDC = hardwareMap.get(DcMotor.class, "Back_Left");
        backRightDC = hardwareMap.get(DcMotor.class, "Back_Right");
        lift = hardwareMap.get(DcMotor.class, "Lift");
        arm = hardwareMap.get(DcMotor.class, "Arm");

        armSwivel = hardwareMap.get(Servo.class, "Arm_Swivel");
        grabber = hardwareMap.get(Servo.class, "Grabber");
        ringGrabber = hardwareMap.get(Servo.class, "Ring_Grabber");

        frontLeftDC.setDirection(DcMotor.Direction.FORWARD);
        backLeftDC.setDirection(DcMotor.Direction.FORWARD);
        frontRightDC.setDirection(DcMotor.Direction.REVERSE);
        backRightDC.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ringGrabberOpen = false;

        wheelMultiplier = 1;

        telemetry.addData("Init", "Complete");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.right_stick_button && r3Timer.milliseconds() > 250) {
            if (wheelMultiplier == 1) {
                wheelMultiplier = 0.25;
            } else {
                wheelMultiplier = 1;
            }

            r3Timer.reset();
        }

        frontLeftDC.setPower(((gamepad1.left_stick_y - gamepad1.left_stick_x) - gamepad1.right_stick_x) * wheelMultiplier);
        backLeftDC.setPower(((gamepad1.left_stick_y + gamepad1.left_stick_x) - gamepad1.right_stick_x) * wheelMultiplier);
        frontRightDC.setPower(((gamepad1.left_stick_y + gamepad1.left_stick_x) + gamepad1.right_stick_x) * wheelMultiplier);
        backRightDC.setPower(((gamepad1.left_stick_y - gamepad1.left_stick_x) + gamepad1.right_stick_x) * wheelMultiplier);

        if (gamepad1.right_bumper) {
            arm.setPower(0.5);
        } else if (gamepad1.left_bumper) {
            arm.setPower(-0.5);
        } else {
            arm.setPower(0);
        }

        if (gamepad1.right_trigger > 0.0) {
            lift.setPower(gamepad1.right_trigger);

            if ((lift.getCurrentPosition() > MAX_LIFT_POSITION)) {
                lift.setPower(-0.3);
                lift.setTargetPosition(MAX_LIFT_POSITION);
            }
        } else if (gamepad1.left_trigger > 0.0) {
            lift.setPower(-gamepad1.left_trigger);

            if ((lift.getCurrentPosition() < MIN_LIFT_POSITION)) {
                lift.setPower(0.3);
                lift.setTargetPosition(MIN_LIFT_POSITION);
            }
        } else {
            lift.setPower(0);
        }

        if (gamepad1.a && aTimer.milliseconds() > 250) {
            if (ringGrabberOpen) {
                ringGrabber.setPosition(0);
                ringGrabberOpen = false;
            } else {
                ringGrabber.setPosition(1);
                ringGrabberOpen = true;
            }
            aTimer.reset();
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
            grabber.setPosition(0.5);
        }

        if (gamepad1.y) {
            Point centerOfHighGoal = aimAssist.getCenterOfHighGoal();
        }

        telemetry.addData("Front Left Motor Speed", frontLeftDC.getPower());
        telemetry.addData("Back Left Motor Speed", backLeftDC.getPower());
        telemetry.addData("Front Right Motor Speed", frontRightDC.getPower());
        telemetry.addData("Back Right Motor Speed", backRightDC.getPower());
        telemetry.addData("Arm swivel:", armSwivel.getPosition());
        telemetry.addData("Grabber", grabber.getPosition());
        telemetry.addData("Lift position:", lift.getCurrentPosition());
        telemetry.addData("Ring grabber position:", ringGrabber.getPosition());
        telemetry.addData("Ring grabber arm:", arm.getCurrentPosition());
        telemetry.addData("Wheel multiplier:", wheelMultiplier);
        telemetry.update();
    }
}
