package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * Op mode for preliminary tuning of the follower PID coefficients (located in the drive base
 * classes). The robot drives back and forth in a straight line indefinitely. Utilization of the
 * dashboard is recommended for this tuning routine. To access the dashboard, connect your computer
 * to the RC's WiFi network. In your browser, navigate to https://192.168.49.1:8080/dash if you're
 * using the RC phone or https://192.168.43.1:8080/dash if you are using the Control Hub. Once
 * you've successfully connected, start the program, and your robot will begin moving forward and
 * backward. You should observe the target position (green) and your pose estimate (blue) and adjust
 * your follower PID coefficients such that you follow the target position as accurately as possible.
 * If you are using SampleMecanumDrive, you should be tuning TRANSLATIONAL_PID and HEADING_PID.
 * If you are using SampleTankDrive, you should be tuning AXIAL_PID, CROSS_TRACK_PID, and HEADING_PID.
 * These coefficients can be tuned live in dashboard.
 *
 * This opmode is designed as a convenient, coarse tuning for the follower PID coefficients. It
 * is recommended that you use the FollowerPIDTuner opmode for further fine tuning.
 */
@Config
@Autonomous(group = "ukdrive")
public class AutoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard.start();
        final SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d blueStart = new Pose2d(-60, 25, 0);
        Pose2d estimatePosition = drive.getPoseEstimate();

        Pose2d square1 = new Pose2d(12, 60, 0);
        Pose2d square2 = new Pose2d(35, 35, 0);
        Pose2d square3 = new Pose2d(50, 60, 0);

        RingDetectionEasyOpenCV ringCount = new RingDetectionEasyOpenCV(hardwareMap, 4, 5);

        final Trajectory trajectoryToSquare1 = drive.trajectoryBuilder(blueStart)
                .splineToLinearHeading(square1, 90)
                .build();
        final Trajectory trajectoryToSquare2 = drive.trajectoryBuilder(blueStart)
                .splineToLinearHeading(square2, 90)
                .build();
        final Trajectory trajectoryToSquare3 = drive.trajectoryBuilder(blueStart)
                .splineToLinearHeading(square3, 90)
                .build();

        waitForStart();

        ringCount.start(new RingDetectionCallback() {
            @Override
            public void noRings() {
                telemetry.addData("rings: ", 0);
                drive.followTrajectory(trajectoryToSquare1);
            }

            @Override
            public void oneRing() {
                telemetry.addData("rings: ", 1);
                drive.followTrajectory(trajectoryToSquare2);
            }

            @Override
            public void fourRings() {
                telemetry.addData("rings: ", 4);
                drive.followTrajectory(trajectoryToSquare3);
            }
        });

        while (opModeIsActive()) {
            telemetry.addData("hue mean:", ringCount.getHueMean());
            telemetry.update();
        }
    }
}