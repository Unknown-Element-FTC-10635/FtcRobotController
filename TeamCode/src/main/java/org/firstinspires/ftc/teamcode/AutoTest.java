package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d blueStart = new Pose2d(-60, 25, 0);
        Pose2d estimatePosition = drive.getPoseEstimate();

        Vector2d square1 = new Vector2d(12, 60);
        Vector2d square2 = new Vector2d(35, 35);
        Vector2d square3 = new Vector2d(60, -60);

        RingDetectionEasyOpenCV ringCount = new RingDetectionEasyOpenCV();

        int numberOfRings = ringCount.getRingCount();

        Trajectory trajectoryToSquare1 = drive.trajectoryBuilder(blueStart)
                .splineTo(square1, 0)
                .build();
        Trajectory trajectoryToSquare2 = drive.trajectoryBuilder(blueStart)
                .splineTo(square2, 0)
                .build();
        Trajectory trajectoryToSquare3 = drive.trajectoryBuilder(blueStart)
                .splineTo(square3, 0)
                .build();

        waitForStart();

        if (numberOfRings == 4) {
            drive.followTrajectory(trajectoryToSquare3);
        } else if (numberOfRings == 1) {
            drive.followTrajectory(trajectoryToSquare2);
        } else {
            drive.followTrajectory(trajectoryToSquare1);
        }


    }
}