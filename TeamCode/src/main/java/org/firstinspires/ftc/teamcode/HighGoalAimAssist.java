package org.firstinspires.ftc.teamcode;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class HighGoalAimAssist {
    public void createContours(String importFile, String exportFile) {
        Mat frame = Imgcodecs.imread(importFile);

        //Converting the source image to binary
        Mat gray = new Mat(frame.rows(), frame.cols(), frame.type());

        Imgproc.cvtColor(frame, gray, Imgproc.COLOR_BGR2GRAY);
        Imgproc.blur(gray, gray, new Size(5, 5));

        Mat binary = new Mat(frame.rows(), frame.cols(), frame.type(), new Scalar(0));
        Imgproc.threshold(gray, binary, 15, 255, Imgproc.THRESH_BINARY_INV);

        //Finding Contours
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchey = new Mat();
        Imgproc.findContours(binary, contours, hierarchey, Imgproc.RETR_TREE,
                Imgproc.CHAIN_APPROX_SIMPLE);

        //Drawing the Contours
        Scalar color = new Scalar(0, 1, 0);
        Imgproc.drawContours(binary, contours, -1, color, 2, Imgproc.LINE_8,
                hierarchey, 2, new Point());

        Imgcodecs.imwrite(exportFile, binary);
    }
}
