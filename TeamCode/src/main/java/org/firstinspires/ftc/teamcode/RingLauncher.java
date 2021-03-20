package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RingLauncher {
    DcMotorEx launch1, launch2;
    Servo flicker, leftLinkage, rightLinkage;

    final double SERVO_POSITION_OUT = 0.425;
    final double SERVO_POSITION_IN = 0.292;
    final double LEFT_LINKAGE_OUT = 0.9064;
    final double RIGHT_LINKAGE_OUT = 0.0449;

    private boolean ringsLaunching;
    int ringsFired = 0;

    int targetVelocity = 0;

    ElapsedTime servoTimer = new ElapsedTime();
    private int targetRPM;
    public RingLauncher(HardwareMap hardwareMap) {
        launch1 = hardwareMap.get(DcMotorEx.class, "launch1");
        launch2 = hardwareMap.get(DcMotorEx.class, "launch2");
        leftLinkage = hardwareMap.get(Servo.class, "leftLinkage");
        rightLinkage = hardwareMap.get(Servo.class, "rightLinkage");
        flicker = hardwareMap.get(Servo.class, "flicker");

        launch2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(140, 10, 0, 0));
        launch1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(140, 10, 0, 0));
    }

    public void launchHandler(int rings) {
        ringsLaunching = true;
        launch();
        if (ringsFired >= rings) {
            while (servoTimer.milliseconds() < 130) {}//in time
            launch1.setVelocity(0);
            launch2.setVelocity(0);
            flicker.setPosition(SERVO_POSITION_OUT);
            ringsLaunching = false;
            ringsFired = 0;
        }
        return;
    }

    private void launch() {
        RingLauncherState launcherState = RingLauncherState.INIT;
        targetVelocity = (targetRPM / 90) * 28;
        switch (launcherState) {
            case INIT: // init
                leftLinkage.setPosition(LEFT_LINKAGE_OUT);
                rightLinkage.setPosition(RIGHT_LINKAGE_OUT);
                flicker.setPosition(SERVO_POSITION_OUT);
                launcherState = RingLauncherState.SPIN_UP;
                break;
            case SPIN_UP: // spin up
                if (launch1.getVelocity() >= targetVelocity) {
                    launcherState = RingLauncherState.LOAD;
                }
                break;
            case LOAD: // servo out
                if (servoTimer.milliseconds() > 150) {//in time
                    servoTimer.reset();
                    launcherState = RingLauncherState.FIRE;
                    flicker.setPosition(SERVO_POSITION_OUT);
                }
                break;
            case FIRE: // servo in
                if (servoTimer.milliseconds() > 150) {//out time
                    servoTimer.reset();
                    launcherState = RingLauncherState.SPIN_UP;
                    flicker.setPosition(SERVO_POSITION_IN);
                    ringsFired++;
                }
                break;
        }

        launch1.setVelocity(targetVelocity);
        launch2.setVelocity(targetVelocity);
    }

    public void spinUpFlyWheel (int velocity) {
        velocity = (velocity / 90) * 28;
        launch1.setVelocity(velocity);
        launch2.setVelocity(velocity);
    }

    public void setTargetRPM(int targetRPM) {
        this.targetRPM = targetRPM;
    }

    public int getRPM() {
        return  (int) (60 * (launch1.getVelocity() / 28.0) * 1.5);
    }

    public boolean ringsLaunching() {
        return ringsLaunching;
    }

    public void launchRings() {
        ringsLaunching = true;
    }

    public enum RingLauncherState {
        INIT,
        SPIN_UP,
        LOAD,
        FIRE
    }
}
