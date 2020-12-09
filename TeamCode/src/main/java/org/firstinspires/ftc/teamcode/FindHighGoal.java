package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class FindHighGoal {
    public Mat findHighGoal(String importFile, String exportFile) {
        Mat frame = Imgcodecs.imread(importFile);
        MatOfPoint bigBlueThing = findBigBlueThing(frame);
        Imgproc.drawContours(frame, Arrays.asList(bigBlueThing), -1, new Scalar(0, 255, 0), 10, Imgproc.LINE_8);
        Mat highGoal = findHighGoal(frame, bigBlueThing);

        Imgcodecs.imwrite(exportFile, highGoal);

        return highGoal;
    }

    private Mat findHighGoal(Mat input, MatOfPoint contour) {
        Point[] points = contour.toArray();
        Point smallestX = new Point(Integer.MAX_VALUE, 0);
        Point smallestY = new Point(0, Integer.MAX_VALUE);
        Point largestX = new Point(0, 0);
        Point largestY = new Point(0, 0);

        for (Point point : points) {
            if (point.x < smallestX.x) {
                smallestX = point;
            }
            if (point.x > largestX.x) {
                largestX = point;
            }
            if (point.y > largestY.y) {
                largestY = point;
            }
            if (point.y < smallestY.y) {
                smallestY = point;
            }
        }

        double centerX = (smallestX.x + ((largestX.x - smallestX.x) / 2));

        System.out.printf("largest x: %s\n", largestX);

        Imgproc.circle(input, smallestX, 8, new Scalar(0, 0, 255), -1);
        Imgproc.circle(input, largestX, 8, new Scalar(0, 0, 255), -1);
        Imgproc.circle(input, largestY, 8, new Scalar(0, 0, 255), -1);

        Imgproc.line(input, new Point(smallestX.x, largestY.y), new Point(largestX.x, largestY.y), new Scalar(0, 0, 255), 10);
        Imgproc.circle(input, new Point(centerX, largestY.y), 8, new Scalar(255, 0, 0), -1);

        return input;
    }

    private MatOfPoint findBigBlueThing(Mat input) {
        Mat frame = new Mat();
        Imgproc.blur(input, frame, new Size(7, 7));

        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_BGR2HSV);

        Core.inRange(frame, new Scalar(109, 70, 50), new Scalar(180, 255, 255), frame);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(frame, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        Collections.sort(contours, new Comparator<MatOfPoint>() {
            @Override
            public int compare(MatOfPoint o1, MatOfPoint o2) {
                double area1 = Imgproc.contourArea(o1);
                double area2 = Imgproc.contourArea(o2);
                return (int) (area2 - area1);
            }
        });

        List<MatOfPoint> sublist = contours.subList(0, 1);
        System.out.printf("sublist length: %d\n", sublist.size());

        return contours.get(0);
    }
}
