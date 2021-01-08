package org.firstinspires.ftc.teamcode;

import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;

public class HighGoalAimAssistTest {

    @BeforeClass
    public static void setUp() {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }

    @Test
    public void imageContours() {
        Mat frame = new Mat();
        frame = Imgcodecs.imread("./src/test/resources/FourRings.jpeg");


        FindHighGoal highGoal = new FindHighGoal();
        highGoal.findHighGoal(frame);
    }
}
