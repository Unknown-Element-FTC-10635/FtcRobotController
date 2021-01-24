package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Point;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;

@TeleOp(name = "ukdrive")
public class EllieTeleOpMode extends OpMode {

    ExpansionHubMotor launch1, launch2, intake,  wobble;
    ExpansionHubServo flicker, leftLinkage, rightLinkage, gripper;

    DcMotor frontLeft, frontRight, backLeft, backRight;

    private double wheelMultiplier;

   // private AimAssistPipeline aimAssist;

    private ElapsedTime r3Timer = new ElapsedTime();
    private ElapsedTime bTimer = new ElapsedTime();
    private ElapsedTime optionsTimer = new ElapsedTime();

    private final Point RING_LAUNCH_POINT = new Point(320, 240);

    int rpm;
    int targetRPM = 3800;
    int targetPowerShotRPM = 2500;

    boolean launcherEnable = false;
    boolean powershotEnable = false;
    double idlePower = 0.58;

    final double SERVO_OUT = 0.425;
    final double SERVO_IN = 0.292;
    final double LEFT_LINKAGE_IN = 0.27;
    final double LEFT_LINKAGE_OUT = 0.9064;
    final double RIGHT_LINKAGE_IN = 0.7084;
    final double RIGHT_LINKAGE_OUT = 0.0449;
    final double GRIPPER_CLOSED = 0.55;
    final double GRIPPER_OPEN = 0;

    private boolean grabberOpen = false;
    private boolean intakeReversed = false;
    private boolean intakeOff = true;
    private boolean intakeIn = true;

    double servoPosition = 0.5;
    int launcherState = -1;
    ElapsedTime servoTimer = new ElapsedTime();

    @Override
    public void init() {
       // aimAssist = new AimAssistPipeline(hardwareMap);
       // aimAssist.start();

        launch1 = hardwareMap.get(ExpansionHubMotor.class, "launch1");
        launch2 = hardwareMap.get(ExpansionHubMotor.class, "launch2");
        intake = hardwareMap.get(ExpansionHubMotor.class, "intake");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        wobble = hardwareMap.get(ExpansionHubMotor.class, "wobble");
//        launch1.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDCoefficients(140, 10, 0));
//        launch2.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDCoefficients(140, 10, 0));

        wobble.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launch1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launch2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobble.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flicker = hardwareMap.get(ExpansionHubServo.class, "flicker");
        leftLinkage = hardwareMap.get(ExpansionHubServo.class, "leftLinkage");
        rightLinkage = hardwareMap.get(ExpansionHubServo.class, "rightLinkage");
        gripper = hardwareMap.get(ExpansionHubServo.class, "gripper");

        wheelMultiplier = 1;

        telemetry.addData("Init", "Complete");
        telemetry.update();
    }

    @Override
    public void loop() {
        // 7*4 ticks per rev, 1.5 gear ratio, 60 seconds
        rpm = (int) (60 * (launch1.getVelocity() / 28.0) * 1.5);

        // Enable and Disable Slowmode
        if (gamepad1.right_stick_button && r3Timer.milliseconds() > 250) {
            if (wheelMultiplier == 1) {
                wheelMultiplier = 0.25;
            } else {
                wheelMultiplier = 1;
            }

            r3Timer.reset();
        }

        // Movement
        frontLeft.setPower(((gamepad1.left_stick_y - gamepad1.left_stick_x) - gamepad1.right_stick_x) * wheelMultiplier);
        backLeft.setPower(((gamepad1.left_stick_y + gamepad1.left_stick_x) - gamepad1.right_stick_x) * wheelMultiplier);
        frontRight.setPower(-((gamepad1.left_stick_y + gamepad1.left_stick_x) + gamepad1.right_stick_x) * wheelMultiplier);
        backRight.setPower(-((gamepad1.left_stick_y - gamepad1.left_stick_x) + gamepad1.right_stick_x) * wheelMultiplier);

        // Increase TargetRPM
        if (gamepad1.right_bumper) {
            targetRPM += 1;
        }

        // Decrease TargetRPM
        if (gamepad1.left_bumper) {
            targetRPM -= 1;
        }

        // Arm Up and Down
        wobble.setPower((gamepad1.left_trigger - gamepad1.right_trigger) * .5);

        // Grabber
        if (gamepad1.b && bTimer.milliseconds() > 250) {
            bTimer.reset();
            if (grabberOpen) {
                gripper.setPosition(GRIPPER_CLOSED);
                grabberOpen = false;
            } else {
                gripper.setPosition(GRIPPER_OPEN);
                grabberOpen = true;
            }
        }

        // Intake Direction
        if (gamepad1.dpad_down && !intakeOff) {
            intake.setPower(1);
        }

        if (gamepad1.dpad_left && !intakeOff) {
            intake.setPower(-1);
        }

        // Intake on or off
        if (gamepad1.dpad_up) {
            if (intakeOff) {
                intake.setPower(-1);
                intakeOff = false;
            } else {
                intake.setPower(0);
            }
        }

        // Open close intake
        if (gamepad1.options && optionsTimer.milliseconds() > 250) {
            optionsTimer.reset();
            if (intakeIn) {
                leftLinkage.setPosition(LEFT_LINKAGE_OUT);
                rightLinkage.setPosition(RIGHT_LINKAGE_OUT);
                intakeIn = false;
            } else {
                leftLinkage.setPosition(LEFT_LINKAGE_IN);
                rightLinkage.setPosition(RIGHT_LINKAGE_IN);
                intakeIn = true;
            }
        }

        // Powershot launcher
        if (gamepad1.a) {
            powershotEnable = true;
            launcherState = 0;
        }

        // Enable launcher
        if (gamepad1.y) {
            launcherState = 0;
        }

        // Launcher
        switch (launcherState) {
            case 0:
                leftLinkage.setPosition(LEFT_LINKAGE_OUT);
                rightLinkage.setPosition(RIGHT_LINKAGE_OUT);
                launcherState++;
                launcherEnable = true;
                break;
            case 1:
                if ((rpm + 250 > targetRPM) || (rpm + 250 > targetPowerShotRPM))
                    if (powershotEnable) {
                        launcherState = 8;
                    } else {
                        launcherState++;
                    }
                break;
            case 2:
                servoTimer.reset();
                launcherState++;
                flicker.setPosition(SERVO_IN);
                break;
            case 3:
            case 5:
            case 7:
            case 9:
                if (servoTimer.milliseconds() > 120) {//in time
                    servoTimer.reset();
                    launcherState++;
                    flicker.setPosition(SERVO_OUT);
                }
                break;
            case 4:
            case 6:
            case 8:
                if (servoTimer.milliseconds() > 120) {//out time
                    servoTimer.reset();
                    launcherState++;
                    flicker.setPosition(SERVO_IN);
                }
                break;
            case 10:
                if (servoTimer.milliseconds() > 120) {//in time
                    launcherEnable = false;
                    powershotEnable = false;
                    launch1.setPower(0);
                    launch2.setPower(0);
                    launcherState++;
                }
                break;
        }

        if (launcherEnable) {
            if (rpm < targetRPM || rpm < targetPowerShotRPM) {
                if (rpm < targetRPM - 250 || rpm < targetPowerShotRPM - 250) {
                    launch1.setPower(1);
                    launch2.setPower(1);
                } else {
                    launch1.setPower(idlePower);
                    launch2.setPower(idlePower);
                }
                idlePower += 0.0003;
            } else {
                launch1.setPower(idlePower);
                launch2.setPower(idlePower);
                idlePower -= 0.0003;
            }
        }
        /*
        // TODO: aim-assist
        if (gamepad1.y) {
            Point centerOfHighGoal = aimAssist.getCenterOfHighGoal();
        } */

        telemetry.addData("Wheel multiplier:", wheelMultiplier);
        telemetry.addData("RPM:", rpm);
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Timer", servoTimer.milliseconds());

        telemetry.update();
    }
}
