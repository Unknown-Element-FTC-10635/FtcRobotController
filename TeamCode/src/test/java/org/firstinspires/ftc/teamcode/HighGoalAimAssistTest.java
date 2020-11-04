package org.firstinspires.ftc.teamcode;

import org.junit.Test;
import org.opencv.core.Core;

public class HighGoalAimAssistTest {
    @Test
    public void imageContours() {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
        String importFile = "./src/test/resources/FourRings.jpeg";
        String exportFile = "./build/ContrastContours.jpeg";

        HighGoalAimAssist aimAssist = new HighGoalAimAssist();
        aimAssist.findHighGoal(importFile, exportFile);
    }
}
