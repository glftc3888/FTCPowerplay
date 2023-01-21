package org.firstinspires.ftc.teamcode.Autonomous;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

public class SleeveDetection extends OpenCvPipeline {
    /*
    YELLOW  = Parking Left
    CYAN    = Parking Middle
    MAGENTA = Parking Right
     */
    // Width and height for the bounding box
    public static int REGION_WIDTH = 50;
    public static int REGION_HEIGHT = 70;

    public enum ParkingPosition {
        LEFT,
        CENTER,
        RIGHT,
        NOTHING,
    }

    // TOPLEFT anchor point for the bounding box
    private static Point SLEEVE_TOPLEFT_ANCHOR_POINT = new Point(320/2 - REGION_WIDTH/3,240/2 - REGION_HEIGHT/2);



    // Lower and upper boundaries for colors
    private static final Scalar
            // 20 60 90
            // lower cyan: leave at 0, 55, ~60 (super dark)
            // lower yellow: 120, 80, 30
            // lower magenta: 100, 20, 45
            lower_yellow_bounds  = new Scalar(110, 70, 0, 255),
            upper_yellow_bounds  = new Scalar(255, 255, 130, 255),
            lower_cyan_bounds    = new Scalar(0, 70, 70, 255),
            upper_cyan_bounds    = new Scalar(130, 255, 255, 255),
            lower_magenta_bounds = new Scalar(90, 0, 40, 255),
            upper_magenta_bounds = new Scalar(255, 120, 255, 255);

    // Color definitions
    private final Scalar
            YELLOW  = new Scalar(255, 255, 0),
            CYAN    = new Scalar(0, 255, 255),
            MAGENTA = new Scalar(255, 0, 255);

    // Percent and mat definitions
    private double yelPercent, cyaPercent, magPercent;

    //telemetry
    private Scalar avgScalar = new Scalar(0, 0, 0, 0);
    private Mat yelMat = new Mat(), cyaMat = new Mat(), magMat = new Mat(), blurredMat = new Mat(), kernel = new Mat();

    // Anchor point definitions
    Point sleeve_pointA = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y);
    Point sleeve_pointB = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    // Running variable storing the parking position
    private volatile ParkingPosition position = ParkingPosition.NOTHING;

    @Override
    public Mat processFrame(Mat input) {
        // Noise reduction
        Imgproc.blur(input, blurredMat, new Size(5, 5));
        blurredMat = blurredMat.submat(new Rect(sleeve_pointA, sleeve_pointB));
        
        // Apply Morphology
        kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.morphologyEx(blurredMat, blurredMat, Imgproc.MORPH_CLOSE, kernel);
        
        // Gets channels from given source mat
        Core.inRange(blurredMat, lower_yellow_bounds, upper_yellow_bounds, yelMat);
        Core.inRange(blurredMat, lower_cyan_bounds, upper_cyan_bounds, cyaMat);
        Core.inRange(blurredMat, lower_magenta_bounds, upper_magenta_bounds, magMat);

        // Gets color specific values
        yelPercent = Core.countNonZero(yelMat);
        cyaPercent = Core.countNonZero(cyaMat);
        magPercent = Core.countNonZero(magMat);

        // Calculates the highest amount of pixels being covered on each side
        double maxPercent = Math.max(yelPercent, Math.max(cyaPercent, magPercent));

        // Checks all percentages, will highlight bounding box in camera preview
        // based on what color is being detected
        if (maxPercent == yelPercent) {
            position = ParkingPosition.LEFT;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    YELLOW,
                    2
            );
        } else if (maxPercent == cyaPercent) {
            position = ParkingPosition.CENTER;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    CYAN,
                    2
            );
        } else if (maxPercent == magPercent) {
            position = ParkingPosition.RIGHT;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    MAGENTA,
                    2
            );
        }
        
        // Memory cleanup
        blurredMat.release();
        yelMat.release();
        cyaMat.release();
        magMat.release();
        kernel.release();

        return input;
    }

    // Returns an enum being the current position where the robot will park
    public ParkingPosition getPosition() {
        return position;
    }

    public double yelPercent(){
        return yelPercent;
    }
    public double magPercent(){
        return magPercent;
    }
    public double cyaPercent(){
        return cyaPercent;
    }
}
