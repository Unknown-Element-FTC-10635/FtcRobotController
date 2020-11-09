package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

        Pose2d blueStart = new Pose2d(-60, 18, 0);
        Vector2d avoidRingsPoint = new Vector2d(-10, 24);

        final Vector2d square1 = new Vector2d(0, 60);
        final Vector2d square2 = new Vector2d(20, 38);
        final Vector2d square3 = new Vector2d(47, 65);

        final RingDetectionEasyOpenCV ringCount = new RingDetectionEasyOpenCV(hardwareMap, 3, 5);

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

        drive.setPoseEstimate(blueStart);

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        ringCount.start(new RingDetectionCallback() {

            @Override
            public void ringsCounted(int numberOfRings) throws InterruptedException {
                telemetry.addData("rings: ", numberOfRings);
                telemetry.addData("Hue mean:", ringCount.getHueMean());
                telemetry.update();

                switch (numberOfRings) {
                    case 0:
                        drive.followTrajectory(trajectoryToSquare1);
                        afterRingCount(new Pose2d(5, 50, 0), new Pose2d(square1.getX(), square1.getY()));
                        break;

                    case 1:
                        drive.followTrajectory(trajectoryToSquare2);
                        afterRingCount(new Pose2d(20, 34, 0), new Pose2d(square2.getX(), square2.getY()));
                        break;

                    case 4:
                        drive.followTrajectory(trajectoryToSquare3);
                        afterRingCount(new Pose2d(60, 40, 0), new Pose2d(square3.getX(), square3.getY()));
                        break;
                }
            }
        });

        while (opModeIsActive() && !isStopRequested()) {

        }
    }

    private void afterRingCount(Pose2d destination, Pose2d currentSquare) throws InterruptedException {
        Vector2d blueSecondWobblePosition = new Vector2d(-53, 36);
        Pose2d poseSecondWobble = new Pose2d(-51, 37, 0);
        Vector2d reverseFromPosition = new Vector2d(-10, 24);

        Trajectory navToSecondWobble = drive.trajectoryBuilder(currentSquare)
                .back(5)
                .splineToConstantHeading(reverseFromPosition, Math.toRadians(180))
                .splineToConstantHeading(blueSecondWobblePosition, Math.toRadians(180))
                .build();
        Trajectory returnToSquare = drive.trajectoryBuilder(poseSecondWobble)
                .splineToLinearHeading(destination, 0)
                .build();

        armSwivel.setPosition(0.5);

        Thread.sleep(500);

        grabber.setPosition(1);

        Thread.sleep(500);

        armSwivel.setPosition(0);
        drive.followTrajectory(navToSecondWobble);

        Thread.sleep(1500);

        grabber.setPosition(0);

        Thread.sleep(1000);

        drive.followTrajectory(returnToSquare);
        grabber.setPosition(1);
    }
}