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

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(group = "ukdrive")
public class AutoTest extends LinearOpMode {
    Servo armSwivel;
    Servo grabber;
    SampleMecanumDrive drive;

    @Override
    public void runOpMode() {
        FtcDashboard.start();
        drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armSwivel = hardwareMap.get(Servo.class, "Arm_Swivel");
        grabber = hardwareMap.get(Servo.class, "Grabber");

        Pose2d blueStart = new Pose2d(-63, 18, 0);
        Vector2d avoidRingsPoint = new Vector2d(-10, 24);

        final Vector2d square1 = new Vector2d(5, 62);
        final Vector2d square2 = new Vector2d(30, 30);
        final Vector2d square3 = new Vector2d(47, 65);

        final Trajectory trajectoryToSquare1 = drive.trajectoryBuilder(blueStart)
                .splineToConstantHeading(square1, Math.toRadians(90))
                .build();
        final Trajectory trajectoryToSquare2 = drive.trajectoryBuilder(blueStart)
                .lineTo(avoidRingsPoint)
                .splineToConstantHeading(square2, Math.toRadians(90))
                .build();
        final Trajectory trajectoryToSquare3 = drive.trajectoryBuilder(blueStart)
                .lineTo(avoidRingsPoint)
                .splineToConstantHeading(square3, Math.toRadians(90))
                .build();

        final RingDetectionEasyOpenCV ringCount = new RingDetectionEasyOpenCV(hardwareMap, telemetry);

        drive.setPoseEstimate(blueStart);

        telemetry.addLine("Waiting for start");
        telemetry.update();

        ringCount.start();

        waitForStart();

        while (opModeIsActive() && ringCount.getFrameCount() < 60) {
            sleep(100);
        }

        int numberOfRings = ringCount.getRingCount();

        telemetry.addData("rings: ", numberOfRings);
        telemetry.update();

        armSwivel.setPosition(1);

        switch (numberOfRings) {
            case 0:
                drive.followTrajectory(trajectoryToSquare1);
                afterRingCount(new Pose2d(-4, 55, 0), vectorToPose(square1, 0));
                break;

            case 1:
                drive.followTrajectory(trajectoryToSquare2);
                afterRingCount(new Pose2d(30, 42, 0), vectorToPose(square2, 0));
                break;

            case 4:
                drive.followTrajectory(trajectoryToSquare3);
                afterRingCount(new Pose2d(60, 40, 0), vectorToPose(square3, 0));
                break;
        }

        ringCount.stop();

    }

    private void afterRingCount(Pose2d destination, Pose2d currentSquare) {
        Vector2d blueSecondWobblePosition = new Vector2d(-45, 50);
        Vector2d navToSquare1 = new Vector2d(-16, 60);

        Trajectory backUp = drive.trajectoryBuilder(currentSquare, true)
                .strafeTo(navToSquare1)
                .build();
        Trajectory navToSecondWobble = drive.trajectoryBuilder(backUp.end())
                .splineToLinearHeading(vectorToPose(blueSecondWobblePosition, 180), Math.toRadians(180))
                .build();
        Trajectory pickUpSecondWobble = drive.trajectoryBuilder(navToSecondWobble.end())
                .forward(3)
                .build();
        Trajectory rotateBack = drive.trajectoryBuilder(pickUpSecondWobble.end(), true)
                .lineToLinearHeading(vectorToPose(navToSquare1, 0))
                .build();
        Trajectory returnToSquare = drive.trajectoryBuilder(rotateBack.end())
                .splineToLinearHeading(destination, 90)
                .build();
        Trajectory reverseFive = drive.trajectoryBuilder(returnToSquare.end())
                .back(5)
                .build();
        Trajectory parkOnLine = drive.trajectoryBuilder(reverseFive.end())
                .strafeTo(new Vector2d(-4, 55))
                .build();

        armSwivel.setPosition(0.8);

        sleep(500);

        grabber.setPosition(1);

        sleep(500);

        drive.followTrajectory(backUp);
        drive.followTrajectory(navToSecondWobble);
        //drive.followTrajectory(pickUpSecondWobble);

        sleep(1500);

        grabber.setPosition(0);

        sleep(1000);

        drive.followTrajectory(rotateBack);
        drive.followTrajectory(returnToSquare);
        grabber.setPosition(1);

        drive.followTrajectory(parkOnLine);
    }

    private Pose2d vectorToPose(Vector2d vector, int heading) {
        return new Pose2d(vector.getX(), vector.getY(), Math.toRadians(heading));
    }
}