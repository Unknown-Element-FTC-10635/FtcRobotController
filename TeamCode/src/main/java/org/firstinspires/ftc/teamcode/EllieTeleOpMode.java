package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;

@TeleOp(name = "ukdrive")
public class EllieTeleOpMode extends OpMode {
    ExpansionHubMotor launch1, launch2, intake, wobble;
    ExpansionHubServo flicker, leftLinkage, rightLinkage, gripper;

    private double wheelMultiplier = 1;

    // private AimAssistPipeline aimAssist;
    SampleMecanumDrive drive;

    private ElapsedTime r3Timer = new ElapsedTime();
    private ElapsedTime bTimer = new ElapsedTime();
    private ElapsedTime optionsTimer = new ElapsedTime();

    private final int HIGH_GOAL_RPM = 3600;
    private final int POWERSHOT_RPM = 3400;

    int userAdjustedRPM = 0;

    final double LEFT_LINKAGE_IN = 0.27;
    final double LEFT_LINKAGE_OUT = 0.9064;
    final double RIGHT_LINKAGE_IN = 0.7084;
    final double RIGHT_LINKAGE_OUT = 0.0449;
    final double GRIPPER_CLOSED = 0.4;
    final double GRIPPER_OPEN = 0;

    private boolean grabberOpen = false;
    private boolean intakeOff = true;
    private boolean intakeIn = true;

    ElapsedTime servoTimer = new ElapsedTime();

    private boolean lastRightBumperState = false;
    private boolean lastLeftBumperState = false;
    private boolean lastSquareState = false;

    boolean userFire = true;

    RingLauncher ringLauncher;
    private PowerShotFire powerShotFireState = PowerShotFire.STOPPED;

    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(12, 34, 0));
        //drive.setPoseEstimate(new Pose2d(-63, 18, 0));

        ringLauncher = new RingLauncher(hardwareMap);
        // aimAssist = new AimAssistPipeline(hardwareMap);
        // aimAssist.start();

        launch1 = hardwareMap.get(ExpansionHubMotor.class, "launch1");
        launch2 = hardwareMap.get(ExpansionHubMotor.class, "launch2");
        intake = hardwareMap.get(ExpansionHubMotor.class, "intake");
        wobble = hardwareMap.get(ExpansionHubMotor.class, "wobble");

        wobble.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launch1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launch2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        wobble.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flicker = hardwareMap.get(ExpansionHubServo.class, "flicker");
        leftLinkage = hardwareMap.get(ExpansionHubServo.class, "leftLinkage");
        rightLinkage = hardwareMap.get(ExpansionHubServo.class, "rightLinkage");
        gripper = hardwareMap.get(ExpansionHubServo.class, "gripper");

        telemetry.addLine("Init Complete");
        telemetry.update();
    }

    @Override
    public void loop() {
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
        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y * wheelMultiplier,
                        -gamepad1.left_stick_x * wheelMultiplier,
                        -gamepad1.right_stick_x * wheelMultiplier
                )
        );
        drive.update();
        
        if (lastRightBumperState && !gamepad1.right_bumper) {
            userAdjustedRPM += 10;
        }
        lastRightBumperState = gamepad1.right_bumper;

        if (lastLeftBumperState && !gamepad1.left_bumper) {
            userAdjustedRPM -= 10;
        }
        lastLeftBumperState = gamepad1.left_bumper;

        /*
        if (lastSquareState && !gamepad1.x) {
            if (userFire) {
                userFire = false;
            } else {
                userFire = true;
            }
        }
        lastSquareState = gamepad1.x; */

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
            if (userFire) {
                ringLauncher.setTargetRPM(POWERSHOT_RPM + userAdjustedRPM);
                ringLauncher.launch(1);
            } else {
                powerShotFireState = PowerShotFire.FIRST_POSITION;
            }
        }

        switch (powerShotFireState) {
            case FIRST_POSITION:
                Trajectory powershot = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-3, 15, 0))
                        .build();

                powerShotFire(powershot);
                powerShotFireState = PowerShotFire.SECOND_POSITION;
                break;

            case SECOND_POSITION:
                Trajectory powershot2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-3, 9, 0))
                        .build();

                powerShotFire(powershot2);
                powerShotFireState = PowerShotFire.THIRD_POSITION;
                break;

            case THIRD_POSITION:
                Trajectory powershot3 = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-3, 2, 0))
                        .build();

                powerShotFire(powershot3);
                powerShotFireState = PowerShotFire.STOPPED;
                break;

            case STOPPED:
                break;
        }

        // Enable launcher
        if (gamepad1.y) {
            ringLauncher.setTargetRPM(HIGH_GOAL_RPM + userAdjustedRPM);
            ringLauncher.launch(3);
            /*
            if (userFire) {
                ringLauncher.setTargetRPM(HIGH_GOAL_RPM + userAdjustedRPM);
                ringLauncher.launch(3);
            } else {
                Trajectory highGoal = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-4, 38, 0))
                        .build();

                drive.followTrajectory(highGoal);
                ringLauncher.setTargetRPM(HIGH_GOAL_RPM + userAdjustedRPM);
                ringLauncher.launch(3);
            } */
        }

        /*
        if (gamepad1.dpad_right) {
            drive.setPoseEstimate(new Pose2d(-63, 63, 0));
        } */

        /*
        // TODO: aim-assist
        if (gamepad1.y) {
            Point centerOfHighGoal = aimAssist.getCenterOfHighGoal();
        } */

        telemetry.addData("Wheel multiplier:", wheelMultiplier);
        telemetry.addData("RPM:", ringLauncher.getRPM());
        telemetry.addData("High Goal RPM", HIGH_GOAL_RPM + userAdjustedRPM);
        telemetry.addData("Power Shot RPM", POWERSHOT_RPM + userAdjustedRPM);
        telemetry.addData("Timer", servoTimer.milliseconds());
        telemetry.addData("Servo:", gripper.getPosition());
        telemetry.addData("MANUAL", userFire);

        telemetry.update();
    }

    public void powerShotFire(Trajectory trajectory) {
        drive.followTrajectory(trajectory);
        ringLauncher.setTargetRPM(POWERSHOT_RPM);
        ringLauncher.launch(1);

    }
}
