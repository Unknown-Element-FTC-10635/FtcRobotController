package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.revextensions2.ExpansionHubMotor;

@Config
@Autonomous(group = "ukdrive")
public class AutonomousOpMode extends LinearOpMode {
    ExpansionHubMotor wobbleArm, intake;
    Servo grabber, leftLinkage, rightLinkage;
    SampleMecanumDrive drive;

    RingLauncher ringLauncher;

    int targetRPM = 3600;

    final double LEFT_LINKAGE_OUT = 0.9064;
    final double RIGHT_LINKAGE_OUT = 0.0449;

    final double LEFT_LINKAGE_RAISED = 0.75;
    final double RIGHT_LINKAGE_RAISED = 0.15;

    final int ROTATE_OPEN = 800;

    double intakeCurrentDraw = 0;
    double currentThreshold = 3000;
    double intakeNormalSpeed = 0.45;

    @Override
    public void runOpMode() {
        FtcDashboard.start();
        drive = new SampleMecanumDrive(hardwareMap);
        ringLauncher = new RingLauncher(hardwareMap);

        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wobbleArm = hardwareMap.get(ExpansionHubMotor.class, "wobble");
        grabber = hardwareMap.get(Servo.class, "gripper");
        intake = hardwareMap.get(ExpansionHubMotor.class, "intake");

        leftLinkage = hardwareMap.get(Servo.class, "leftLinkage");
        rightLinkage = hardwareMap.get(Servo.class, "rightLinkage");

        wobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //wobbleArm.setPower(0);
        //wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Pose2d blueStart = new Pose2d(-63, 18, 0);
        Vector2d avoidRingsPoint = new Vector2d(-10, 24);

        final Vector2d square1 = new Vector2d(13, 50);
        final Vector2d square2 = new Vector2d(33, 25);
        final Vector2d square3 = new Vector2d(60, 45);

        final Trajectory trajectoryToSquare1 = drive.trajectoryBuilder(blueStart)
                .splineTo(square1, 0)
                .build();
        final Trajectory trajectoryToSquare2 = drive.trajectoryBuilder(blueStart)
                .lineTo(avoidRingsPoint)
                .splineTo(square2, 0)
                .build();
        final Trajectory trajectoryToSquare3 = drive.trajectoryBuilder(blueStart)
                .lineTo(avoidRingsPoint)
                .splineTo(square3, 0)
                .build();

        final RingDetectionEasyOpenCV ringCount = new RingDetectionEasyOpenCV(hardwareMap, telemetry);

        drive.setPoseEstimate(blueStart);

        telemetry.addLine("Waiting for start");
        telemetry.update();

        ringCount.start();

        waitForStart();

        grabber.setPosition(0.4);

        while (opModeIsActive() && ringCount.getFrameCount() < 60) {
            sleep(100);
        }

        int numberOfRings = ringCount.getRingCount();

        telemetry.addData("rings: ", numberOfRings);
        telemetry.update();

        wobbleArm.setTargetPosition(ROTATE_OPEN);
        wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleArm.setPower(0.1);

        switch (numberOfRings) {
            case 0:
                drive.followTrajectory(trajectoryToSquare1);
                afterRingCount(new Pose2d(10, 45, 0), vectorToPose(square1, 0), 0);
                break;

            case 1:
                drive.followTrajectory(trajectoryToSquare2);
                afterRingCount(new Pose2d(30, 18, 0), vectorToPose(square2, 0), 1);
                break;

            case 4:
                drive.followTrajectory(trajectoryToSquare3);
                afterRingCount(new Pose2d(55, 42, 0), vectorToPose(square3, 0), 4);
                break;
        }

        ringCount.stop();

    }

    private void afterRingCount(Pose2d destination, Pose2d currentSquare, int rings) {
        Trajectory reverseFromWobbles = drive.trajectoryBuilder(currentSquare)
                .back(15)
                .build();
        Trajectory firingPosition = drive.trajectoryBuilder(currentSquare)
                .lineToLinearHeading(new Pose2d(-5, 34, 0))
                .build();
        Trajectory powershot1 = drive.trajectoryBuilder(currentSquare)
                .lineTo(new Vector2d(-3, 18))
                .build();
        /*Trajectory powershot2 = drive.trajectoryBuilder(powershot1.end())
                .lineTo(new Vector2d(-3, 10))
                .build();
        Trajectory powershot3 = drive.trajectoryBuilder(powershot2.end())
                .lineTo(new Vector2d(-3, 3))
                .build();*/
        Trajectory secondWobble = drive.trajectoryBuilder(powershot1.end(), true)
                .lineToLinearHeading(new Pose2d(-50, 25, 0))
                .build();
        Trajectory strefeLeft = drive.trajectoryBuilder(secondWobble.end())
                .strafeLeft(10)
                .build();
        Trajectory forward = drive.trajectoryBuilder(strefeLeft.end())
                .forward(17)
                .build();
        Trajectory back = drive.trajectoryBuilder(forward.end())
                .back(5)
                .build();
        Trajectory intakeCollection = drive.trajectoryBuilder(back.end())
                .forward(20)
                .build();
        Trajectory getAwayFromRings = drive.trajectoryBuilder(intakeCollection.end())
                .strafeRight(20)
                .build();
        Trajectory dropOffSecondWobble = drive.trajectoryBuilder(getAwayFromRings.end())
                .lineToLinearHeading(destination)
                .build();
        Trajectory getAwayFromSecondWobble = drive.trajectoryBuilder(dropOffSecondWobble.end())
                .strafeRight(5)
                .build();
        Trajectory getAwayFromSecondWobbleFirstPosition = drive.trajectoryBuilder(dropOffSecondWobble.end())
                .strafeRight(20)
                .build();
        Trajectory firingPositionAfterSecondWobble = drive.trajectoryBuilder(getAwayFromSecondWobble.end())
                .lineToLinearHeading(new Pose2d(-5, 34, 0))
                .build();
        Trajectory parkOnLine = drive.trajectoryBuilder(firingPosition.end())
                .lineToLinearHeading(new Pose2d(4, 34))
                .build();

//        wobbleArm.setTargetPosition(rotateToOpened);
//        wobbleArm.setPower(0.3);
//        wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // while (wobbleArm.getCurrentPosition() < ROTATE_OPEN - 20) {}

        grabber.setPosition(0);
        //sleep(100);

        wobbleArm.setPower(0.4);
        wobbleArm.setTargetPosition(0);
       // drive.followTrajectory(reverseFromWobbles);

        drive.followTrajectory(firingPosition);
        ringLauncher.setTargetRPM(targetRPM);
        ringLauncher.launch(3);

        /*
        drive.followTrajectory(powershot1);

        // ring launcher
        ringLauncher.setTargetRPM(targetRPM);
        ringLauncher.launch(1);

        drive.followTrajectory(powershot2);

        // ring launcher
        ringLauncher.setTargetRPM(targetRPM);
        ringLauncher.launch(1);

        drive.followTrajectory(powershot3);

        // ring launcher
        ringLauncher.setTargetRPM(targetRPM);
        ringLauncher.launch(1); */

        wobbleArm.setPower(.35);
        wobbleArm.setTargetPosition(ROTATE_OPEN);
        wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive.followTrajectory(secondWobble);

        drive.followTrajectory(strefeLeft);

        grabber.setPosition(.4);

        sleep(1000);

        if (rings == 4) {
            leftLinkage.setPosition(LEFT_LINKAGE_RAISED);
            rightLinkage.setPosition(RIGHT_LINKAGE_RAISED);

            //drive.followTrajectory(forward);

            drive.setMotorPowers(0.2, 0.2, 0.2, 0.2);
            sleep(250);
            drive.setMotorPowers(0, 0, 0, 0);

            drive.followTrajectory(back);

            leftLinkage.setPosition(LEFT_LINKAGE_OUT);
            rightLinkage.setPosition(RIGHT_LINKAGE_OUT);

        }

        if (rings == 4 || rings == 1) {
            intake.setPower(1);
            drive.followTrajectoryAsync(intakeCollection);

            while (drive.isBusy()) {
                adjustVoltage();
                drive.update();
            }

            drive.followTrajectoryAsync(getAwayFromRings);

            while (drive.isBusy()) {
                adjustVoltage();
                drive.update();
            }
        }
        drive.followTrajectoryAsync(dropOffSecondWobble);

        while (drive.isBusy()) {
            adjustVoltage();
            drive.update();
        }

        grabber.setPosition(0);

        sleep(200);

        wobbleArm.setTargetPosition(600);
        drive.followTrajectory(getAwayFromSecondWobble);

        if (rings == 0) {
            drive.followTrajectory(getAwayFromSecondWobbleFirstPosition);
        }
        wobbleArm.setTargetPosition(0);

        if (rings == 1 || rings == 4) {
            ringLauncher.setTargetRPM(targetRPM);
            ringLauncher.spinUpFlyWheel();
            drive.followTrajectory(firingPositionAfterSecondWobble);
            ringLauncher.launch(3);
        }

        drive.followTrajectory(parkOnLine);

    }

    private Pose2d vectorToPose(Vector2d vector, int heading) {
        return new Pose2d(vector.getX(), vector.getY(), Math.toRadians(heading));
    }

    private void adjustVoltage() {
        intakeCurrentDraw = intake.getCurrent(CurrentUnit.MILLIAMPS);
        if (intakeCurrentDraw > currentThreshold && intake.getPower() > 0.4 && intake.getPower() < 0.9) {
            intake.setPower(1);
        } else if (intakeCurrentDraw < currentThreshold && intake.getPower() > 0.9) {
            intake.setPower(intakeNormalSpeed);
        }
    }
}