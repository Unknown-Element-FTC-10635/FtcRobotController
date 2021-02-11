package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;

@Config
@Autonomous(group = "ukdrive")
public class AutonomousOpMode extends LinearOpMode {
    DcMotor wobbleArm, intake;
    Servo grabber, leftLinkage, rightLinkage;
    SampleMecanumDrive drive;

    RingLauncher ringLauncher;

    int targetRPM = 3660;

    final double LEFT_LINKAGE_OUT = 0.9064;
    final double RIGHT_LINKAGE_OUT = 0.0449;

    final double LEFT_LINKAGE_RAISED = 0.75;
    final double RIGHT_LINKAGE_RAISED = 0.15;

    final int ROTATE_OPEN = 800;


    @Override
    public void runOpMode() {
        FtcDashboard.start();
        drive = new SampleMecanumDrive(hardwareMap);
        ringLauncher = new RingLauncher(hardwareMap);

        wobbleArm = hardwareMap.get(DcMotor.class, "wobble");
        grabber = hardwareMap.get(Servo.class, "gripper");
        intake = hardwareMap.get(DcMotor.class, "intake");

        leftLinkage = hardwareMap.get(Servo.class, "leftLinkage");
        rightLinkage = hardwareMap.get(Servo.class, "rightLinkage");

        wobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //wobbleArm.setPower(0);
        //wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Pose2d blueStart = new Pose2d(-63, 18, 0);
        Vector2d avoidRingsPoint = new Vector2d(-10, 24);

        final Vector2d square1 = new Vector2d(10, 45);
        final Vector2d square2 = new Vector2d(30, 20);
        final Vector2d square3 = new Vector2d(55, 45);

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

        grabber.setPosition(0.35);

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
                afterRingCount(new Pose2d(-4, 55, 0), square1);
                break;

            case 1:
                drive.followTrajectory(trajectoryToSquare2);
                afterRingCount(new Pose2d(25, 41, 0), square2);
                break;

            case 4:
                drive.followTrajectory(trajectoryToSquare3);
                afterRingCount(vectorToPose(square3, 0), square3);
                break;
        }

        ringCount.stop();

    }

    private void afterRingCount(Pose2d destination, Vector2d currentSquare) {
        Trajectory reverseFromWobbles = drive.trajectoryBuilder(vectorToPose(currentSquare, 0))
                .back(15)
                .build();
        Trajectory firingPosition = drive.trajectoryBuilder(reverseFromWobbles.end())
                .strafeTo(new Vector2d(-3, 34))
                .build();
        Trajectory powershot1 = drive.trajectoryBuilder(reverseFromWobbles.end())
                .lineTo(new Vector2d(-3, 18))
                .build();
        /*Trajectory powershot2 = drive.trajectoryBuilder(powershot1.end())
                .lineTo(new Vector2d(-3, 10))
                .build();
        Trajectory powershot3 = drive.trajectoryBuilder(powershot2.end())
                .lineTo(new Vector2d(-3, 3))
                .build();*/
        Trajectory secondWobble = drive.trajectoryBuilder(powershot1.end(), true)
                .splineToConstantHeading(new Vector2d(-50, 25), 0)
                .build();
        Trajectory strefeLeft = drive.trajectoryBuilder(secondWobble.end())
                .strafeLeft(8)
                .build();
        Trajectory forward = drive.trajectoryBuilder(strefeLeft.end())
                .forward(15)
                .build();
        Trajectory back = drive.trajectoryBuilder(forward.end())
                .back(5)
                .build();
        Trajectory intakeCollection = drive.trajectoryBuilder(back.end())
                .forward(25)
                .build();
        Trajectory getAwayFromRings = drive.trajectoryBuilder(intakeCollection.end())
                .strafeRight(25)
                .build();

        Trajectory parkOnLine = drive.trajectoryBuilder(reverseFromWobbles.end())
                .strafeTo(new Vector2d(12, 34))
                .build();

//        wobbleArm.setTargetPosition(rotateToOpened);
//        wobbleArm.setPower(0.3);
//        wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (wobbleArm.getCurrentPosition() < ROTATE_OPEN - 20) {}

        grabber.setPosition(0);
        sleep(100);

        wobbleArm.setTargetPosition(0);
        drive.followTrajectory(reverseFromWobbles);

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

        drive.followTrajectory(secondWobble);

        wobbleArm.setPower(.35);
        wobbleArm.setTargetPosition(ROTATE_OPEN);
        wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (wobbleArm.getCurrentPosition() < ROTATE_OPEN - 20) {}

        drive.followTrajectory(strefeLeft);

        grabber.setPosition(.35);
        leftLinkage.setPosition(LEFT_LINKAGE_RAISED);
        rightLinkage.setPosition(RIGHT_LINKAGE_RAISED);

        drive.followTrajectory(forward);
        drive.followTrajectory(back);

        leftLinkage.setPosition(LEFT_LINKAGE_OUT);
        rightLinkage.setPosition(RIGHT_LINKAGE_OUT);

        intake.setPower(1);

        drive.followTrajectory(intakeCollection);

        drive.followTrajectory(getAwayFromRings);
        drive.followTrajectory(drive.trajectoryBuilder(getAwayFromRings.end()).strafeTo(currentSquare).build());
        grabber.setPosition(0);
        wobbleArm.setTargetPosition(0);
        drive.followTrajectory(reverseFromWobbles);

        drive.followTrajectory(parkOnLine);

    }

    private Pose2d vectorToPose(Vector2d vector, int heading) {
        return new Pose2d(vector.getX(), vector.getY(), Math.toRadians(heading));


    }
}