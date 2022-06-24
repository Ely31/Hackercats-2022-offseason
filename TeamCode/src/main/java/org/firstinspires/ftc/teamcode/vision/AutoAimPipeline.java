package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class AutoAimPipeline extends OpenCvPipeline {
    // Camera resolution
    private final double cameraWidth = 320;
    private final double cameraHeight = 240;

    // Lower and upper limits for the threshold
    public static Scalar lower = new Scalar(55, 140, 58);
    public static Scalar upper = new Scalar(120, 200, 130);

    private Mat thresholdedMat = new Mat();
    List<MatOfPoint> contours = new ArrayList<>();
    private Mat outputMat = new Mat();

    private final Scalar contourColor = new Scalar(0,255,0);
    private final Scalar rectangleColor = new Scalar(0,0,255);

    private Rect bestRect = new Rect();

    @Override
    public Mat processFrame(Mat input) {
        outputMat = input.clone();
        Imgproc.cvtColor(input, thresholdedMat, Imgproc.COLOR_RGB2YCrCb); // Convert to YCrCb color space
        Imgproc.blur(thresholdedMat, thresholdedMat, new Size(3,3)); // Blur to get rid of noise
        Core.inRange(thresholdedMat, lower,upper, thresholdedMat); // Threshold

        // Find contours
        Imgproc.findContours(thresholdedMat, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        // Draw contours
        Imgproc.drawContours(outputMat, contours, -1, contourColor, 1);

        for (MatOfPoint contour : contours) {
            Point[] contourArray = contour.toArray();
            if (contourArray.length > 15) {
                MatOfPoint2f areaPoints = new MatOfPoint2f(contourArray);
                Rect rect = Imgproc.boundingRect(areaPoints);
                Imgproc.rectangle(outputMat, rect, rectangleColor, 1);
            }
        }



        contours.clear(); // Clear the contours each loop so they don't pile up on top of the old ones
        return outputMat;
    }
}
