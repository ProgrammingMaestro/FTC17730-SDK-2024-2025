package vision;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.opencv.dnn.Dnn;
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

public class ColorDetectionPipeline extends OpenCvPipeline {
    boolean shouldRet = true;
    private Mat hsvImage = new Mat();
    private Mat mask = new Mat();
    private Mat dilatedMask = new Mat();

    private List<MatOfPoint> contours = new ArrayList<>();

    // Color thresholds in RGB
    private final Scalar LOWER_RED = new Scalar(0, 100, 100);
    private final Scalar UPPER_RED = new Scalar(255, 255, 255);
    private final Scalar LOWER_GREEN = new Scalar(40, 100, 100);
    private final Scalar UPPER_GREEN = new Scalar(80, 255, 255);
    private final Scalar LOWER_BLUE = new Scalar(100, 50, 80);
    private final Scalar UPPER_BLUE = new Scalar(15, 200, 255);

    @Override
    public Mat processFrame(Mat frame) {
        // Check if the frame is empty
        if (frame.empty()) {
            return frame; // Return empty frame
        }
        mask.release();

        contours.clear();
        // Convert the frame to HSV
        Imgproc.cvtColor(frame, hsvImage, Imgproc.COLOR_BGR2HSV);

        // Create a mask for detected colors
        mask.setTo(new Scalar(0)); // Reset mask
        Core.inRange(hsvImage, LOWER_RED, UPPER_RED, mask);
        Mat maskGreen = new Mat();
        Core.inRange(hsvImage, LOWER_GREEN, UPPER_GREEN, maskGreen);
        Core.add(mask, maskGreen, mask);
        Mat maskBlue = new Mat();
        Core.inRange(hsvImage, LOWER_BLUE, UPPER_BLUE, maskBlue);
        Core.add(mask, maskBlue, mask);

        // Apply morphological operations to clean the mask
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new org.opencv.core.Size(5, 5));
        Imgproc.dilate(mask, dilatedMask, kernel);  // Dilation to enhance features
        Imgproc.erode(dilatedMask, mask, kernel);   // Erosion to reduce noise

        // Find contours in the mask
        Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Draw contours and bounding boxes
        for (MatOfPoint contour : contours) {
            Rect rect = Imgproc.boundingRect(contour);
            if ( rect.area() > 100) { //Max & Min area threshold to filter out noise
                Imgproc.rectangle(frame, rect.tl(), rect.br(), new Scalar(0, 255, 0), 1); // Draw rectangle

                // Draw text near the rectangle
                String label = "Detected"; // Label for the detected object
                int fontFace = Imgproc.FONT_HERSHEY_SIMPLEX;
                double fontScale = 0.5;
                int thickness = 1;
                Point textOrg = new Point(rect.x, rect.y - 5); // Position the text above the rectangle

                Imgproc.putText(frame, label, textOrg, fontFace, fontScale, new Scalar(255, 0, 0), thickness);
            }
        }

        // Return the original frame with drawn rectangles and text
        return frame;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        //Draw things on Frame
    }
}
