package org.firstinspires.ftc.teamcode.Competition_Code.Subsystems;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class GameboardDetectionSystem extends OpenCvPipeline {
    private Point ballCenter = null;

    @Override
    public Mat processFrame(Mat input) {
        // Convert to HSV
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        // Threshold for ball color
        Scalar lower = new Scalar(20, 100, 100);  // example HSV lower bound
        Scalar upper = new Scalar(30, 255, 255);  // example HSV upper bound
        Mat mask = new Mat();
        Core.inRange(hsv, lower, upper, mask);

        // Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, new Mat(),
                Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Pick largest contour
        double maxArea = 0;
        Rect bestRect = null;
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                bestRect = Imgproc.boundingRect(contour);
            }
        }

        if (bestRect != null) {
            ballCenter = new Point(bestRect.x + bestRect.width/2.0,
                    bestRect.y + bestRect.height/2.0);
            // Draw circle for debugging
            Imgproc.circle(input, ballCenter, 10, new Scalar(0,255,0), 2);
        }

        return input; // return annotated frame
    }

    public Point getBallCenter() {
        return ballCenter;
    }
}