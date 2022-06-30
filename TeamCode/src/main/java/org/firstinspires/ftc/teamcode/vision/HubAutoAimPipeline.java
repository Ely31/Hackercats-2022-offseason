package org.firstinspires.ftc.teamcode.vision;

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

public class HubAutoAimPipeline extends OpenCvPipeline {
    // True is normal with the rects and contours, false is the thresholded image
    public boolean outputMode = true;

    // Camera constants
    private final double cameraWidth = 320;
    private final double cameraHeight = 240;
    private final double fov = 42; // Obtained through primitive calibration, kind of concerning how different it is from logitech's specs
    private final double pixelsPerDegreeX = fov/cameraWidth;

    // Lower and upper limits for the threshold
    public static Scalar lower = new Scalar(60, 140, 96);
    public static Scalar upper = new Scalar(255, 212, 147);

    private final int maxHeight = (int) (cameraHeight * (1.0/4.0));
    private final int minHeight = (int) (cameraHeight * 5.0/7.0);

    // Mats and things
    private Mat processedMat = new Mat();
    private List<MatOfPoint> contours = new ArrayList<>();
    Point[] bestRectPoints = new Point[4];
    private Mat outputMat = new Mat();
    private Mat thresholdedMat = new Mat();
    private Mat hierarchy = new Mat();
    private RotatedRect[] fittedRect;

    // Colors
    private final Scalar contourColor = new Scalar(255,0,0);
    private final Scalar rectColor = new Scalar(0,0,255);
    private final Scalar bestRectColor = new Scalar(0,255,0);
    private final Scalar textColor = new Scalar(255,255,255);

    private final double optimalEccentricity = 3.2;
    private final double optimalAreaRatio = 1;
    private final double optimalScore = 0;
    // Set best score really high to start so that it will actually update to things the pipeline finds
    private double bestRectScore = 10;
    // And also set the best rect to something really bad for the same reason
    public RotatedRect bestRect = new RotatedRect(new Point(0,0), new Size(5,50),0);
    public RotatedRect lastRealBestRect = new RotatedRect();

    @Override
    public Mat processFrame(Mat input) {
        // If we don't clear the best rect and score every loop, it will stay stuck on an old rect and score
        resetBestRect();
        bestRectScore = 100;

        // Blur to get rid of noise
        Imgproc.blur(input, processedMat, new Size(2,2));
        processedMat.copyTo(outputMat);
        // Convert to YCrCb color space
        Imgproc.cvtColor(processedMat, processedMat, Imgproc.COLOR_RGB2YCrCb);
        // Threshold
        Core.inRange(processedMat, lower,upper, processedMat);
        processedMat.copyTo(thresholdedMat);
        // Find contours
        Imgproc.findContours(processedMat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        // Draw contours
        Imgproc.drawContours(outputMat, contours, -1, contourColor, 1);

        bestRect.points(bestRectPoints); // To stop an npe from happening while trying to draw them if it finds no best rect

        // Fit an rect to each contour
        for (int i = 0; i < contours.size(); i++) {
            if (contours.get(i).rows() > 25) { // Check if it's more than 15 to filter out small ones
                fittedRect = new RotatedRect[contours.size()];
                fittedRect[i] = Imgproc.minAreaRect(new MatOfPoint2f(contours.get(i).toArray()));

                // Draw rectangles, it's annoyingly hard with the rotated ones
                Point[] rectPoints = new Point[4];
                fittedRect[i].points(rectPoints);
                for (int j = 0; j < 4; j++) {
                    Imgproc.line(outputMat, rectPoints[j], rectPoints[(j+1) % 4], rectColor, 1);
                }

                // If this rect scores higher than the best one and is in the height window, make it the best one
                //  && (fittedRect[i].center.y > minHeight && fittedRect[i].center.y < maxHeight)
                if ((getRectScore(fittedRect[i], contours.get(i)) < bestRectScore)&& (fittedRect[i].center.y < minHeight && fittedRect[i].center.y > maxHeight)) {
                    bestRectScore = getRectScore(fittedRect[i], contours.get(i));
                    bestRect = fittedRect[i];
                    fittedRect[i].points(bestRectPoints);
                    lastRealBestRect = fittedRect[i];
                }
            }
        }
        // Draw height window
        Imgproc.line(outputMat, new Point(0,maxHeight), new Point(cameraWidth, maxHeight), textColor,1);
        Imgproc.line(outputMat, new Point(0,minHeight), new Point(cameraWidth, minHeight), textColor,1);
        // Draw the best rect on the viewport
        for (int j = 0; j < 4; j++) {
            Imgproc.line(outputMat, bestRectPoints[j], bestRectPoints[(j+1) % 4], bestRectColor, 1);
        }
        // Print stats about the rect
        Imgproc.putText(outputMat, "score:" + bestRectScore, bestRect.center, 0, 0.4, textColor);
        Imgproc.putText(outputMat, "eccentricity:" + getEccentricity(bestRect), (new Point(bestRect.center.x, bestRect.center.y+12)), 0, 0.4, textColor);

        contours.clear(); // Clear them each time so they don't stack up

        if (outputMode) return outputMat;
        else return thresholdedMat;
    } // End of proccessFrame

    // Methods
    // Finds how close to a circle an rect is. an output of 1 is a perfect circle.
    private double getEccentricity(RotatedRect rect){
        return Math.max(rect.size.height / rect.size.width, rect.size.width / rect.size.height);
    }
    private double getRectAreaRatio(RotatedRect rect, MatOfPoint contour){
        // Area of rect divided by area of contour
        return ((rect.size.width) * (rect.size.height)) / Imgproc.contourArea(contour);
    }
    // Score = eccentricity + ratio of rect area / contour area. A lower score is better. 0 is a perfect score.
    private double getRectScore(RotatedRect rect, MatOfPoint contour){
        return Math.abs(optimalEccentricity-getEccentricity(rect)) + Math.abs( optimalAreaRatio - getRectAreaRatio(rect, contour));
    }
    private void resetBestRect(){
        bestRect = new RotatedRect(new Point(0,0), new Size(5,50),0);
    }

    // Methods to return the ball coords
    public double getRawX(){
        return lastRealBestRect.center.x;
    }
    public double getRawY(){
        return lastRealBestRect.center.y;
    }
    // Corrected means that if the ball is at the middle of the frame, the output is zero
    public double getCorrectedX(){
        return (getRawX() - (cameraWidth/2));
    }
    public double getCorrectedY(){
        return (getRawY() - (cameraHeight/2));
    }

    // See https://youtu.be/rLwOkAJqImo?t=1828s
    public double getAngle(){
        return -(getCorrectedX() * pixelsPerDegreeX);
    }
}
