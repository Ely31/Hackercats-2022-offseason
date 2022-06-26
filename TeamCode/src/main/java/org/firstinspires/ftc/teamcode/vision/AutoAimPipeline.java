package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class AutoAimPipeline extends OpenCvPipeline {
    // True is normal with the ellipses and contours, false is the thresholded image
    public boolean outputMode = true;

    // Camera resolution
    private final double cameraWidth = 320;
    private final double cameraHeight = 240;

    // Lower and upper limits for the threshold
    public static Scalar lower = new Scalar(55, 125, 40);
    public static Scalar upper = new Scalar(255, 200, 100);

    // Mats and things
    private Mat processedMat = new Mat();
    private List<MatOfPoint> contours = new ArrayList<>();
    private Mat outputMat = new Mat();
    private Mat thresholdedMat = new Mat();
    private Mat hierarchy = new Mat();
    private RotatedRect[] fittedEllipse;

    // Colors
    private final Scalar contourColor = new Scalar(255,0,0);
    private final Scalar ellipseColor = new Scalar(0,0,255);
    private final Scalar bestEllipseColor = new Scalar(0,255,0);
    private final Scalar textColor = new Scalar(0,0,0);

    private double bestEllipseScore = 100; // Set it really high to start so that it will actually update to things the pipeline finds
    // And also set the best ellipse to something really bad for the same reason
    public RotatedRect bestEllipse = new RotatedRect(new Point(20,30), new Size(10,50),0);
    public RotatedRect lastRealBestEllipse = new RotatedRect();

    @Override
    public Mat processFrame(Mat input) {
        // If we don't clear the best ellipse and score every loop, it will stay stuck on an old ellipse and score
        resetBestEllipse();
        bestEllipseScore = 100;

        Imgproc.blur(input, processedMat, new Size(2,2)); // Blur to get rid of noise
        outputMat = processedMat.clone();
        Imgproc.cvtColor(processedMat, processedMat, Imgproc.COLOR_RGB2YCrCb); // Convert to YCrCb color space
        Core.inRange(processedMat, lower,upper, processedMat); // Threshold
        thresholdedMat = processedMat.clone();
        // Find contours
        Imgproc.findContours(processedMat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        // Draw contours
        Imgproc.drawContours(outputMat, contours, -1, contourColor, 1);
        // Fit an ellipse to each contour
        for (int i = 0; i < contours.size(); i++) {
            if (contours.get(i).rows() > 15) { // Check if it's more than 15 to filter out small ones
                fittedEllipse = new RotatedRect[contours.size()];
                fittedEllipse[i] = Imgproc.fitEllipse(new MatOfPoint2f(contours.get(i).toArray()));
                Imgproc.ellipse(outputMat, fittedEllipse[i], ellipseColor, 1); // Display all the ellipses it makes

                // If this ellipse scores higher than the best one, make it the best one
                if (getEllipseScore(fittedEllipse[i], contours.get(i)) < bestEllipseScore) {
                    bestEllipseScore = getEllipseScore(fittedEllipse[i], contours.get(i));
                    bestEllipse = fittedEllipse[i];
                    lastRealBestEllipse = fittedEllipse[i];
                }
            }
        }
        // Draw the best ellipse on the viewport
        Imgproc.ellipse(outputMat, bestEllipse,bestEllipseColor);
        Imgproc.putText(outputMat, "score" + bestEllipseScore, bestEllipse.center, 0, 0.4, textColor);

        contours.clear();

        if (outputMode) return outputMat;
        else return thresholdedMat;
    } // End of proccessFrame

    // Methods
    // Finds how close to a circle an ellipse is. an output of 1 is a perfect circle.
    private double getEccentricity(RotatedRect ellipse){
        return Math.max(ellipse.size.width / ellipse.size.height, ellipse.size.height / ellipse.size.width);
    }
    private double getEllipseArea(RotatedRect ellipse){
        return Math.PI * (ellipse.size.width/2) * (ellipse.size.height/2);
    }
    // Score = eccentricity + ratio of ellipse area / contour area. A lower score is better.
    private double getEllipseScore(RotatedRect ellipse, MatOfPoint contour){
        return getEccentricity(ellipse) + (getEllipseArea(ellipse) / Imgproc.contourArea(contour));
    }
    private void resetBestEllipse(){
        bestEllipse = new RotatedRect(new Point(20,30), new Size(5,50),0);
    }

    // Methods to return the ball coords
    public double getRawX(){
        return lastRealBestEllipse.center.x;
    }
    public double getRawY(){
        return lastRealBestEllipse.center.y;
    }
    // Corrected means that if the ball is at the middle of the frame, the output is zero
    public double getCorrectedX(){
        return (getRawX() - (cameraWidth/2));
    }
    public double getCorrectedY(){
        return (getRawY() - (cameraHeight/2));
    }
}
