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
    // Telemetry stuff, the constructor is needed for some reason
    Telemetry telemetry;
    public AutoAimPipeline(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    public boolean outputMode = true;

    // Camera resolution
    private final double cameraWidth = 320;
    private final double cameraHeight = 240;

    // Lower and upper limits for the threshold
    public static Scalar lower = new Scalar(55, 125, 40);
    public static Scalar upper = new Scalar(255, 200, 100);

    private Mat processedMat = new Mat();
    List<MatOfPoint> contours = new ArrayList<>();
    private Mat outputMat = new Mat();
    private Mat thresholdedMat = new Mat();
    private Mat hierarchy = new Mat();

    private final Scalar contourColor = new Scalar(255,0,0);
    private final Scalar ellipseColor = new Scalar(0,0,255);
    private final Scalar bestEllipseColor = new Scalar(0,255,0);

    private final double targetEccentricity = 1;
    private double bestEllipseScore = 100; // Set it really high to start so that it will actually update to things the pipeline finds
    private RotatedRect bestEllipse = new RotatedRect(new Point(20,30), new Size(10,50),0);

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.blur(input, processedMat, new Size(2,2)); // Blur to get rid of noise
        outputMat = processedMat.clone();
        Imgproc.cvtColor(processedMat, processedMat, Imgproc.COLOR_RGB2YCrCb); // Convert to YCrCb color space
        Core.inRange(processedMat, lower,upper, processedMat); // Threshold
        thresholdedMat = processedMat.clone();

        // Find contours
        Imgproc.findContours(processedMat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        // Draw contours
        Imgproc.drawContours(outputMat, contours, -1, contourColor, 1);

        for (int i = 0; i < contours.size(); i++) {
            if (contours.get(i).rows() > 15) { // Check if it's more than 15 to filter out small ones
                RotatedRect[] fittedEllipse = new RotatedRect[contours.size()];
                fittedEllipse[i] = Imgproc.fitEllipse(new MatOfPoint2f(contours.get(i).toArray()));
                Imgproc.ellipse(outputMat, fittedEllipse[i], ellipseColor, 1); // Display all the ellipses it makes

                // If this ellipse scores higher than the best one, make it the best one
                if (getEllipseScore(fittedEllipse[i], contours.get(i)) < bestEllipseScore) {
                    bestEllipseScore = getEllipseScore(fittedEllipse[i], contours.get(i));
                    bestEllipse = fittedEllipse[i];
                }
            }
        }
        // Draw the best ellipse on the viewport
        Imgproc.ellipse(outputMat, bestEllipse,bestEllipseColor);
        Imgproc.putText(outputMat, "eccentricity" + getEccentricity(bestEllipse), bestEllipse.center, 0, 0.4, new Scalar(0,0,0));
        // Display telemetry
        telemetry.addData("best score", bestEllipseScore);
        telemetry.addData("x", getCorrectedX());
        telemetry.addData("y", getCorrectedY());
        telemetry.update();
        contours.clear();
        if (outputMode) return outputMat;
        else return thresholdedMat;
    } // End of proccessFrame

    // Finds how close to a circle an ellipse is. an output of 1 is a perfect circle.
    private double getEccentricity(RotatedRect ellipse){
        return Math.max(ellipse.size.width / ellipse.size.height, ellipse.size.height / ellipse.size.width);
    }
    private double getEllipseArea(RotatedRect ellipse){
        return Math.PI * (ellipse.size.width/2) * (ellipse.size.height/2);
    }
    // Score  = eccentricity + ratio of ellipse area / contour area. A lower score is better.
    private double getEllipseScore(RotatedRect ellipse, MatOfPoint contour){
        return getEccentricity(ellipse) + (getEllipseArea(ellipse) / Imgproc.contourArea(contour));
    }

    // Methods to return the ball coords
    public double getRawX(){
        return bestEllipse.center.x;
    }
    public double getRawY(){
        return bestEllipse.center.y;
    }
    // Corrected means that if the ball is at the middle of the frame, the output is zero
    public double getCorrectedX(){
        return (getRawX() - (cameraWidth/2));
    }
    public double getCorrectedY(){
        return (getRawY() - (cameraHeight/2));
    }
}
