package org.firstinspires.ftc.teamcode.drive.localizer;

import android.support.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 * Note: this could be optimized significantly with REV bulk reads
 */
// @Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.98425; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 7.01; // in; distance between the left and right wheels, assuming wheels centered on tracking center X axis

    public static double Strafe_Pod_X_OFFSET = -5.74; // in; X offset of the lateral wheel,  may need to be changed based on mat squishyness

    public static double Front_Pods_X_Offset = 3.04;  // in; how far the front tracking wheels are from tracking center, may change depending how squishy the mats are

    public static double Strafe_Pod_Y_Offset = 0.028; // in; Strafe Pod is shimmed slightly towards the right side of the robot

    private DcMotor leftEncoder, rightEncoder, strafeEncoder;

    public ExpansionHubEx hub, hub2;


    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(Front_Pods_X_Offset, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(Front_Pods_X_Offset, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(Strafe_Pod_X_OFFSET, Strafe_Pod_Y_Offset, Math.toRadians(90)) // front
        ));

        leftEncoder = hardwareMap.dcMotor.get("BackLeft");
        rightEncoder = hardwareMap.dcMotor.get("BackRight");
        strafeEncoder = hardwareMap.dcMotor.get("RightIntake");

        hub = hardwareMap.get(ExpansionHubEx.class, "ExpansionHub1");
        hub2 = hardwareMap.get(ExpansionHubEx.class, "ExpansionHub2");
    }

    public static double encoderTicksToInches(int ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        RevBulkData bulkData = hub.getBulkInputData();
        RevBulkData bulkData2 = hub2.getBulkInputData();

        return Arrays.asList(
// because bulk reads are better
                encoderTicksToInches(bulkData.getMotorCurrentPosition(leftEncoder)),
                encoderTicksToInches(-bulkData.getMotorCurrentPosition(rightEncoder)),  // reverse the encoder counts instead of reversing motor direction
                encoderTicksToInches(-bulkData2.getMotorCurrentPosition(strafeEncoder))


//                encoderTicksToInches(leftEncoder.getCurrentPosition()),
//                encoderTicksToInches(rightEncoder.getCurrentPosition()),
//                encoderTicksToInches(strafeEncoder.getCurrentPosition())
        );
    }
}
