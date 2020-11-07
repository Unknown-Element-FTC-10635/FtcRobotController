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
    public void runOpMode() throws InterruptedException {
        FtcDashboard.start();
        drive = new SampleMecanumDrive(hardwareMap);

        armSwivel = hardwareMap.get(Servo.class, "Arm_Swivel");
        grabber = hardwareMap.get(Servo.class, "Grabber");

        Pose2d blueStart = new Pose2d(-60, 25, 0);
        Vector2d avoidRingsPoint = new Vector2d(-10, 24);

        final Pose2d square1 = new Pose2d(0, 60, 0 );
        final Pose2d square2 = new Pose2d(20, 38, 0);
        final Pose2d square3 = new Pose2d(42, 65, 0 );

        RingDetectionEasyOpenCV ringCount = new RingDetectionEasyOpenCV(hardwareMap, 4, 4.5);

        final Trajectory trajectoryToSquare1 = drive.trajectoryBuilder(blueStart)
                .splineToLinearHeading(square1, 90)
                .build();
        final Trajectory trajectoryToSquare2 = drive.trajectoryBuilder(blueStart)
                .lineTo(avoidRingsPoint)
                .splineToLinearHeading(square2, 90)
                .build();
        final Trajectory trajectoryToSquare3 = drive.trajectoryBuilder(blueStart)
                .lineTo(avoidRingsPoint)
                .splineToLinearHeading(square3, 90)
                .build();

        waitForStart();

        ringCount.start(new RingDetectionCallback() {
            @Override
            public void noRings() {
                telemetry.addData("rings: ", 0);
                drive.followTrajectory(trajectoryToSquare1);
                try {
                    afterRingCount(new Pose2d(5, 50, 0), square1);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }

            @Override
            public void oneRing() {
                telemetry.addData("rings: ", 1);
                drive.followTrajectory(trajectoryToSquare2);
                try {
                    afterRingCount(new Pose2d(20, 34, 0), square2);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }

            @Override
            public void fourRings() {
                telemetry.addData("rings: ", 4);
                drive.followTrajectory(trajectoryToSquare3);
                try {
                    afterRingCount(new Pose2d(65, 35,0), square3);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }
        });

        while (opModeIsActive()) {
            telemetry.addData("hue mean:", ringCount.getHueMean());
            telemetry.update();
        }
    }

    private void afterRingCount(Pose2d destination, Pose2d currentSquare) throws InterruptedException {
        Vector2d blueSecondWobblePosition = new Vector2d(-53, 50);
        Pose2d poseSecondWobble = new Pose2d(-51,37,0);
        Pose2d reverseFromPosition = new Pose2d(10, 10, 0);
        Vector2d avoidRings = new Vector2d(-10, 15);

        Trajectory navToSecondWobble = drive.trajectoryBuilder(currentSquare)
                .lineToSplineHeading(reverseFromPosition)
                .splineToConstantHeading(avoidRings, 120)
                .splineToConstantHeading(blueSecondWobblePosition, 180)
                .build();
        Trajectory returnToSquare = drive.trajectoryBuilder(poseSecondWobble)
                .splineToLinearHeading(destination, 0)
                .build();

        grabber.setPosition(180);

        Thread.sleep(500);

        armSwivel.setPosition(-1);
        drive.followTrajectory(navToSecondWobble);

        Thread.sleep(1500);

        grabber.setPosition(0);

        Thread.sleep(1000);

       // armSwivel.setPosition(1);

        drive.followTrajectory(returnToSquare);
        grabber.setPosition(180);
    }
}