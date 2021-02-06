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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;

@Config
@Autonomous(group = "ukdrive")
public class AutonomousOpMode extends LinearOpMode {
    DcMotor  wobbleArm;
    Servo grabber;
    SampleMecanumDrive drive;

    ExpansionHubMotor launch1, launch2;
    ExpansionHubServo flicker, leftLinkage, rightLinkage;

    int rpm;
    int targetRPM = 3800;
    int targetPowerShotRPM = 2900;

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

    int launcherState = -1;
    ElapsedTime servoTimer = new ElapsedTime();


    @Override
    public void runOpMode() {
        FtcDashboard.start();

        drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        wobbleArm = hardwareMap.get(DcMotor.class, "wobble");
        grabber = hardwareMap.get(Servo.class, "gripper");

        launch1 = hardwareMap.get(ExpansionHubMotor.class, "launch1");
        launch2 = hardwareMap.get(ExpansionHubMotor.class, "launch2");
        flicker = hardwareMap.get(ExpansionHubServo.class, "flicker");
        leftLinkage = hardwareMap.get(ExpansionHubServo.class, "leftLinkage");
        rightLinkage = hardwareMap.get(ExpansionHubServo.class, "rightLinkage");

        launch1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launch2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        wobbleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Pose2d blueStart = new Pose2d(-63, 18, 0);
        Vector2d avoidRingsPoint = new Vector2d(-10, 24);

        final Vector2d square1 = new Vector2d(10, 40);
        final Vector2d square2 = new Vector2d(30, 15);
        final Vector2d square3 = new Vector2d(55, 42);

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

        grabber.setPosition(0.55);

        while (opModeIsActive() && ringCount.getFrameCount() < 60) {
            sleep(100);
        }

        int numberOfRings = ringCount.getRingCount();

        telemetry.addData("rings: ", numberOfRings);
        telemetry.update();

        switch (numberOfRings) {
            case 0:
                drive.followTrajectory(trajectoryToSquare1);
                afterRingCount(new Pose2d(-4, 55, 0), vectorToPose(square1, 0));
                break;

            case 1:
                drive.followTrajectory(trajectoryToSquare2);
                afterRingCount(new Pose2d(25, 41, 0), vectorToPose(square2, 0));
                break;

            case 4:
                drive.followTrajectory(trajectoryToSquare3);
                afterRingCount(vectorToPose(square3, 0), vectorToPose(square3, 0));
                break;
        }

        ringCount.stop();

    }

    private void afterRingCount(Pose2d destination, Pose2d currentSquare) {
        Vector2d blueSecondWobblePosition = new Vector2d(-46, 50);
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
                .splineToLinearHeading(destination, 0)
                .build();
        Trajectory reverseFromWobbles = drive.trajectoryBuilder(currentSquare)
                .back(15)
                .build();
        Trajectory parkbehindLine = drive.trajectoryBuilder(reverseFromWobbles.end())
                .strafeTo(new Vector2d(-8, 38))
                .build();
        Trajectory parkOnLine = drive.trajectoryBuilder(parkbehindLine.end())
                .strafeTo(new Vector2d(4, 38))
                .build();

        wobbleArm.setPower(.1);

        sleep(3000);

        grabber.setPosition(0);

        sleep(500);

        wobbleArm.setPower(-.25);

        sleep(500);

        /*
        drive.followTrajectory(backUp);
        drive.followTrajectory(navToSecondWobble);
        //drive.followTrajectory(pickUpSecondWobble);

        sleep(1500);

        grabber.setPosition(0);

        sleep(1000);

        drive.followTrajectory(rotateBack);
        drive.followTrajectory(returnToSquare);
        grabber.setPosition(1);

        sleep(500);
        */

        drive.followTrajectory(reverseFromWobbles);
        wobbleArm.setPower(0);
        drive.followTrajectory(parkbehindLine);
        launcherEnable = true;
        launcherState = 0;

        while (launcherEnable) {
            rpm = (int) (60 * (launch1.getVelocity() / 28.0) * 1.5);

            switch (launcherState) {
                case 0:
                    leftLinkage.setPosition(LEFT_LINKAGE_OUT);
                    rightLinkage.setPosition(RIGHT_LINKAGE_OUT);
                    launcherState++;
                    launcherEnable = true;
                    break;
                case 1:
                    if (rpm + 250 > targetRPM) {
                        launcherState++; }
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
                if (rpm < targetRPM) {
                    if (rpm < targetRPM - 250) {
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
        }

        drive.followTrajectory(parkOnLine);

    }

    private Pose2d vectorToPose(Vector2d vector, int heading) {
        return new Pose2d(vector.getX(), vector.getY(), Math.toRadians(heading));


    }
}