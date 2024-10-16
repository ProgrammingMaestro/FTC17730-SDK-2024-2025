package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
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

public class ColorDetectionPipeline implements VisionProcessor {

    private Mat hsvImage = new Mat();
    private Mat mask = new Mat();
    private List<MatOfPoint> contours = new ArrayList<>();

    // Define the lower and upper bounds for the colors
    public static double x1;
    public static double y1;
    public static double z1;//Scalar(210, 14, 29);
    private static final Scalar LOWER_RED_1 = new Scalar(210, 14, 29);
    private static final Scalar UPPER_RED_1 = new Scalar(10, 255, 255);
    private static final Scalar LOWER_RED_2 = new Scalar(160, 100, 100);
    private static final Scalar UPPER_RED_2 = new Scalar(180, 255, 255);
    private static final Scalar LOWER_BLUE = new Scalar(100, 150, 0);
    private static final Scalar UPPER_BLUE = new Scalar(140, 255, 255);
    private static final Scalar LOWER_YELLOW = new Scalar(171, 82, 4);
    private static final Scalar UPPER_YELLOW = new Scalar(251, 252, 180);

    // Store detected colors and their positions
    private List<String> detectedColors = new ArrayList<>();
    private List<Point> detectedPositions = new ArrayList<>();
    private List<String> lastDetectedColors = new ArrayList<>();
    private List<Point> lastDetectedPositions = new ArrayList<>();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        if (frame != null && !frame.empty()) {
            // Process frame
        // Convert the input frame from BGR to HSV color space
        Imgproc.cvtColor(frame, hsvImage, Imgproc.COLOR_BGR2HSV);

        // Clear previous mask and contours
        mask.release();
        contours.clear();
        detectedColors.clear();
        detectedPositions.clear();
        lastDetectedColors.clear();
        lastDetectedPositions.clear();

        // Create masks for each color
        Mat maskRed1 = new Mat();
        Mat maskRed2 = new Mat();
        Mat maskBlue = new Mat();
        Mat maskYellow = new Mat();

        Core.inRange(hsvImage, LOWER_RED_1, UPPER_RED_1, maskRed1);
        Core.inRange(hsvImage, LOWER_RED_2, UPPER_RED_2, maskRed2);
        Core.inRange(hsvImage, LOWER_BLUE, UPPER_BLUE, maskBlue);
        Core.inRange(hsvImage, LOWER_YELLOW, UPPER_YELLOW, maskYellow);

        // Combine the red masks
        Core.bitwise_or(maskRed1, maskRed2, mask);
        // Combine all masks
        Core.bitwise_or(mask, maskBlue, mask);
        Core.bitwise_or(mask, maskYellow, mask);

        // Find contours
        Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Loop through each contour
        for (MatOfPoint contour : contours) {
            Rect rect = Imgproc.boundingRect(contour);
            if (rect.width > 20 && rect.height > 10) { // Filter out small contours
                Imgproc.rectangle(frame, rect.tl(), rect.br(), new Scalar(0, 255, 0), 1);
                ColorResult result = detectColor(hsvImage.submat(rect));
                String colorType = result.colorType;
                Scalar meanColor = result.meanColor;

                // Store the mean color for all detections
                lastDetectedColors.add(colorType);
                lastDetectedPositions.add(new Point(rect.x + rect.width / 2, rect.y)); // Center the text

                // Store the current scalar value for display
                detectedColors.add(String.format("H: %.0f, S: %.0f, V: %.0f", meanColor.val[0], meanColor.val[1], meanColor.val[2]));
                }
            }
        }

        return frame; // Return the processed frame
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint paint = new Paint();
        paint.setColor(Color.GREEN);
        paint.setTextSize(25 * scaleCanvasDensity);

        // Avoid flickering by displaying only last known colors
        for (int i = 0; i < lastDetectedColors.size(); i++) {
            String colorText = lastDetectedColors.get(i);
            Point position = lastDetectedPositions.get(i);

            // Draw color text
            float colorTextX = (float) position.x * scaleBmpPxToCanvasPx;
            float colorTextY = (float) position.y * scaleBmpPxToCanvasPx - 10 * scaleCanvasDensity;
            canvas.drawText(colorText, colorTextX, colorTextY, paint);

            // Draw the current Scalar value below the color text with a fixed offset
            String scalarText = detectedColors.get(i);
            float scalarTextY = colorTextY + 40 * scaleCanvasDensity; // Adjust the offset as needed
            canvas.drawText(scalarText, colorTextX, scalarTextY, paint);
        }
    }



    private ColorResult detectColor(Mat roi) {
            // Calculate the average color in the region of interest (ROI)
            Mat hsvRoi = new Mat();
            if (hsvRoi != null) hsvRoi.release();
            Imgproc.cvtColor(roi, hsvRoi, Imgproc.COLOR_BGR2HSV);
            Scalar meanColor = Core.mean(hsvRoi);

            // Check which color the mean color belongs to
            String colorType = "Unknown";
            if (isColorInRange(meanColor, LOWER_RED_1, UPPER_RED_1) || isColorInRange(meanColor, LOWER_RED_2, UPPER_RED_2)) {
                colorType = "Red";
            } else if (isColorInRange(meanColor, LOWER_BLUE, UPPER_BLUE)) {
                colorType = "Blue";
            } else if (isColorInRange(meanColor, LOWER_YELLOW, UPPER_YELLOW)) {
                colorType = "Yellow";
            }

            return new ColorResult(colorType, meanColor);
        }

    private boolean isColorInRange(Scalar color, Scalar lower, Scalar upper) {
        return color.val[0] >= lower.val[0] && color.val[0] <= upper.val[0] &&
                color.val[1] >= lower.val[1] && color.val[1] <= upper.val[1] &&
                color.val[2] >= lower.val[2] && color.val[2] <= upper.val[2];
    }
}
