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

    private final int TARGET_RPM = 3650;
    private final int POWERSHOT_RPM = 3450;

    final double LEFT_LINKAGE_OUT = 0.9064;
    final double RIGHT_LINKAGE_OUT = 0.0449;

    final double LEFT_LINKAGE_RAISED = 0.75;
    final double RIGHT_LINKAGE_RAISED = 0.15;

    final int ROTATE_OPEN = 800;

    double intakeCurrentDraw = 0;
    double currentThreshold = 3000;
    double intakeNormalSpeed = 0.45;

    final Vector2d SQUARE_1 = new Vector2d(8, 46);
    final Vector2d SQUARE_2 = new Vector2d(35, 23);
    final Vector2d SQUARE_3 = new Vector2d(58, 43);

    Trajectory reverseFromWobblesZero, reverseFromWobbleOne, firingPositionFour, firstPowershotZero,
            firstPowershotOne, powershot2, powershot3, secondWobble, strefeLeft, forward, back, intakeCollection,
            getAwayFromRings, dropOffSecondWobbleZero, dropOffSecondWobbleOne, dropOffSecondWobbleFour,
            getAwayFromSecondWobbleZero, getAwayFromSecondWobbleOne, getAwayFromSecondWobbleFour,
            firingPositionAfterSecondWobble, parkOnLine;

    @Override
    public void runOpMode() {
        FtcDashboard.start();
        drive = new SampleMecanumDrive(hardwareMap);
        ringLauncher = new RingLauncher(hardwareMap);

        final RingDetectionEasyOpenCV ringCount = new RingDetectionEasyOpenCV(hardwareMap, telemetry);
        ringCount.start();

        createPaths();

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
        Pose2d avoidRingsPoint = new Pose2d(-10, 24, 0);

        final Trajectory trajectoryToSquare1 = drive.trajectoryBuilder(blueStart)
                .splineTo(SQUARE_1, 0)
                .build();
        final Trajectory trajectoryToSquare2 = drive.trajectoryBuilder(blueStart)
                .lineToLinearHeading(avoidRingsPoint)
                .splineTo(SQUARE_2, 0)
                .build();
        final Trajectory trajectoryToSquare3 = drive.trajectoryBuilder(blueStart)
                .lineToLinearHeading(avoidRingsPoint)
                .splineToLinearHeading(vectorToPose(SQUARE_3, 0), 0)
                .build();

        drive.setPoseEstimate(blueStart);

        telemetry.addLine("Waiting for start");
        telemetry.update();

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
                grabber.setPosition(0);
                afterRingCount(0);
                break;

            case 1:
                drive.followTrajectory(trajectoryToSquare2);
                grabber.setPosition(0);
                afterRingCount(1);
                break;

            case 4:
                drive.followTrajectory(trajectoryToSquare3);
                grabber.setPosition(0);
                afterRingCount(4);
                break;
        }

        ringCount.stop();

    }

    private void afterRingCount(int rings) {
//        wobbleArm.setTargetPosition(rotateToOpened);
//        wobbleArm.setPower(0.3);
//        wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // while (wobbleArm.getCurrentPosition() < ROTATE_OPEN - 20) {}

        //sleep(100);

        wobbleArm.setPower(0.4);

        if (rings == 4) {
            wobbleArm.setTargetPosition(500);
        }

        if (rings == 0) {
            drive.followTrajectory(reverseFromWobblesZero);
        } else if (rings == 1) {
            drive.followTrajectory(reverseFromWobbleOne);
        }

        //ringLauncher.spinUpFlyWheel(POWERSHOT_RPM);

        if (rings == 4) {
            drive.followTrajectory(firingPositionFour);
            ringLauncher.setTargetRPM(TARGET_RPM);
            ringLauncher.launch(3);
        }

        if (rings != 4) {
            if (rings == 0) {
                drive.followTrajectory(firstPowershotZero);
            } else if (rings == 1) {
                drive.followTrajectory(firstPowershotOne);
            }

            // ring launcher
            ringLauncher.setTargetRPM(POWERSHOT_RPM);
            ringLauncher.launch(1);

            drive.followTrajectory(powershot2);

            // ring launcher
            ringLauncher.setTargetRPM(POWERSHOT_RPM);
            ringLauncher.launch(1);

            drive.followTrajectory(powershot3);

            // ring launcher
            ringLauncher.setTargetRPM(POWERSHOT_RPM);
            ringLauncher.launch(1);
        }

        wobbleArm.setPower(.35);
        wobbleArm.setTargetPosition(ROTATE_OPEN);
        wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive.followTrajectory(secondWobble);

        drive.followTrajectory(strefeLeft);

        grabber.setPosition(.4);

        sleep(250);

        if (rings == 4) {
            leftLinkage.setPosition(LEFT_LINKAGE_RAISED);
            rightLinkage.setPosition(RIGHT_LINKAGE_RAISED);

            //drive.followTrajectory(forward);

            ElapsedTime timer = new ElapsedTime();

            drive.setMotorPowers(0.2, 0.2, 0.2, 0.2);
            while(timer.milliseconds() < 250) {
                drive.update();
            }
            drive.setMotorPowers(0, 0, 0, 0);

            drive.followTrajectory(back);

            leftLinkage.setPosition(LEFT_LINKAGE_OUT);
            rightLinkage.setPosition(RIGHT_LINKAGE_OUT);

            wobbleArm.setTargetPosition(600);
        }

        if (rings == 4 || rings == 1) {
            intake.setPower(1);
            drive.followTrajectory(intakeCollection);
        }

        if (rings == 4) {
            drive.followTrajectory(getAwayFromRings);
            drive.followTrajectory(dropOffSecondWobbleFour);
        } else if (rings == 1) {
            drive.followTrajectory(dropOffSecondWobbleOne);
        } else if (rings == 0) {
            drive.followTrajectory(dropOffSecondWobbleZero);
        }

        grabber.setPosition(0);

        sleep(200);

        wobbleArm.setTargetPosition(600);

        if (rings == 0) {
            drive.followTrajectory(getAwayFromSecondWobbleZero);
        } else if (rings == 1) {
            drive.followTrajectory(getAwayFromSecondWobbleOne);
        } else if (rings == 4) {
            drive.followTrajectory(getAwayFromSecondWobbleFour);
        }

        wobbleArm.setTargetPosition(0);

        if (rings == 1 || rings == 4) {
            //ringLauncher.spinUpFlyWheel(TARGET_RPM);
            drive.followTrajectory(firingPositionAfterSecondWobble);
            ringLauncher.setTargetRPM(TARGET_RPM);
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

    private void createPaths() {
        reverseFromWobblesZero = drive.trajectoryBuilder(vectorToPose(SQUARE_1, 0))
                .lineToLinearHeading(new Pose2d(vectorToPose(SQUARE_1, 0).getX(), vectorToPose(SQUARE_1, 0).getY() - 10, 0))
                .build();
        reverseFromWobbleOne =  drive.trajectoryBuilder(vectorToPose(SQUARE_2, 0))
                .lineToLinearHeading(new Pose2d(vectorToPose(SQUARE_2, 0).getX(), vectorToPose(SQUARE_2, 0).getY() - 10, 0))
                .build();

        firingPositionFour = drive.trajectoryBuilder(vectorToPose(SQUARE_3, 0))
                .lineToLinearHeading(new Pose2d(-3, 36, 0))
                .build();

        firstPowershotZero = drive.trajectoryBuilder(reverseFromWobblesZero.end())
                .lineToLinearHeading(new Pose2d(-4, 18, 0))
                .build();
        firstPowershotOne = drive.trajectoryBuilder(reverseFromWobbleOne.end())
                .lineToLinearHeading(new Pose2d(-4, 18, 0))
                .build();

        powershot2 = drive.trajectoryBuilder(firstPowershotZero.end())
                .lineToLinearHeading(new Pose2d(-4, 10, 0))
                .build();
        powershot3 = drive.trajectoryBuilder(powershot2.end())
                .lineToLinearHeading(new Pose2d(-4, 3, 0 ))
                .build();
        secondWobble = drive.trajectoryBuilder(powershot3.end(), true)
                .lineToLinearHeading(new Pose2d(-50, 25, 0))
                .build();
        strefeLeft = drive.trajectoryBuilder(secondWobble.end())
                .strafeLeft(10)
                .build();
        forward = drive.trajectoryBuilder(strefeLeft.end())
                .forward(17)
                .build();
        back = drive.trajectoryBuilder(forward.end())
                .back(5)
                .build();
        intakeCollection = drive.trajectoryBuilder(back.end())
                .forward(20)
                .build();
        getAwayFromRings = drive.trajectoryBuilder(intakeCollection.end())
                .strafeRight(15)
                .build();

        dropOffSecondWobbleZero = drive.trajectoryBuilder(getAwayFromRings.end())
                .lineToLinearHeading(new Pose2d(10, 45, 0))
                .build();
        dropOffSecondWobbleOne = drive.trajectoryBuilder(getAwayFromRings.end())
                .lineToLinearHeading(new Pose2d(29, 20, 0))
                .build();
        dropOffSecondWobbleFour = drive.trajectoryBuilder(getAwayFromRings.end())
                .lineToLinearHeading(new Pose2d(55, 42, 0))
                .build();

        getAwayFromSecondWobbleZero = drive.trajectoryBuilder(dropOffSecondWobbleZero.end())
                .strafeRight(5)
                .build();
        getAwayFromSecondWobbleOne = drive.trajectoryBuilder(dropOffSecondWobbleOne.end())
                .strafeRight(5)
                .build();
        getAwayFromSecondWobbleFour = drive.trajectoryBuilder(dropOffSecondWobbleFour.end())
                .strafeRight(5)
                .build();

        firingPositionAfterSecondWobble = drive.trajectoryBuilder(getAwayFromSecondWobbleOne.end())
                .lineToLinearHeading(new Pose2d(-5, 34, 0))
                .build();
        parkOnLine = drive.trajectoryBuilder(firingPositionAfterSecondWobble.end())
                .lineToLinearHeading(new Pose2d(5, 34))
                .build();

    }
}