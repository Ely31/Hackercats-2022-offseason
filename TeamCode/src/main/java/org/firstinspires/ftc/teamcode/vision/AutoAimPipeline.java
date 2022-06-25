package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
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

    // Camera resolution
    private final double cameraWidth = 320;
    private final double cameraHeight = 240;

    // Lower and upper limits for the threshold
    public static Scalar lower = new Scalar(55, 140, 58);
    public static Scalar upper = new Scalar(120, 200, 130);

    private Mat thresholdedMat = new Mat();
    List<MatOfPoint> contours = new ArrayList<>();
    private Mat outputMat = new Mat();

    private final Scalar contourColor = new Scalar(255,0,0);
    private final Scalar ellipseColor = new Scalar(0,0,255);
    private final Scalar bestEllipseColor = new Scalar(0,255,0);

    private final double targetEccentricity = 1;
    private double bestEccentricity = 1000; // Set it really high to start so that it will actually update to things the pipeline finds
    private RotatedRect bestEllipse = new RotatedRect();

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.blur(input, thresholdedMat, new Size(2,2)); // Blur to get rid of noise
        outputMat = thresholdedMat.clone();
        Imgproc.cvtColor(thresholdedMat, thresholdedMat, Imgproc.COLOR_RGB2YCrCb); // Convert to YCrCb color space
        Core.inRange(thresholdedMat, lower,upper, thresholdedMat); // Threshold

        // Find contours
        Imgproc.findContours(thresholdedMat, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        // Draw contours
        Imgproc.drawContours(outputMat, contours, -1, contourColor, 1);

        for (int i = 0; i < contours.size(); i++) {
            if (contours.get(i).rows() > 15) { // Check if it's more than 15 to filter out small ones
                RotatedRect[] minEllipse = new RotatedRect[contours.size()];
                minEllipse[i] = Imgproc.fitEllipse(new MatOfPoint2f(contours.get(i).toArray()));

                if (isEllipseBetter(minEllipse[i])){
                    bestEccentricity = getEccentricity(minEllipse[i]);
                    bestEllipse = minEllipse[i];
                }
                Imgproc.ellipse(outputMat, minEllipse[i], ellipseColor, 1); // Display all the ellipses it makes
            }
        }
        // Draw best ellipse on the viewport
        Imgproc.ellipse(outputMat, bestEllipse,bestEllipseColor);
        Imgproc.putText(outputMat, "x:" + getCorrectedX(), bestEllipse.center, 0, 0.4, new Scalar(255,255,255));
        // Display telemetry
        telemetry.addData("contour array", contours.size());
        telemetry.addData("best eccentricity", bestEccentricity);
        telemetry.addData("x", getCorrectedX());
        telemetry.addData("y", getCorrectedY());
        telemetry.update();
        contours.clear(); // Clear the contours each loop so they don't pile up on top of the old ones
        return outputMat;
    } // End of proccessFrame

    // Finds how close to a circle an ellipse is. an output of 1 is a perfect circle.
    private double getEccentricity(RotatedRect ellipse){
        return ellipse.size.width / ellipse.size.height;
    }
    private boolean isEllipseBetter(RotatedRect ellipse){
        return (Math.abs(getEccentricity(ellipse) - targetEccentricity)) < bestEccentricity;
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
