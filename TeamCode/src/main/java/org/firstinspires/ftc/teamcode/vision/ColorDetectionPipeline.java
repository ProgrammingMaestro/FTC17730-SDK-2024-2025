    package org.firstinspires.ftc.teamcode.vision;

    import android.graphics.Canvas;
    import android.graphics.Color;
    import android.graphics.Paint;

    import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
    import org.firstinspires.ftc.vision.VisionProcessor;
    import org.opencv.core.Core;
    import org.opencv.core.CvType;
    import org.opencv.core.Mat;
    import org.opencv.core.MatOfPoint;
    import org.opencv.core.Point;
    import org.opencv.core.Rect;
    import org.opencv.core.Scalar;
    import org.opencv.imgproc.Imgproc;
    import org.openftc.easyopencv.OpenCvPipeline;
    import java.util.ArrayList;
    import java.util.List;

    public class ColorDetectionPipeline extends OpenCvPipeline implements VisionProcessor {

        private Mat hsvImage = new Mat();
        private Mat mask = new Mat();
        private List<MatOfPoint> contours = new ArrayList<>();

        // Scalar boundaries for color thresholding in HSV
        private final Scalar LOWER_RED_1 = new Scalar(200, 51, 51);
        private final Scalar UPPER_RED_1 = new Scalar(255, 0, 0);
        private final Scalar LOWER_BLUE = new Scalar(0, 102, 200);
        private final Scalar UPPER_BLUE = new Scalar(0, 0, 255);
        private final Scalar LOWER_YELLOW = new Scalar(60, 100, 0);
        private final Scalar UPPER_YELLOW = new Scalar(120, 255, 200);

        private Mat maskRed = new Mat();
        private Mat maskBlue = new Mat();
        private Mat maskYellow = new Mat();
        private Mat hierarchy = new Mat();
        Mat hsvRoi = new Mat();

        // Limit number of contours processed (e.g., max 6)
        private int maxContours = 6;

        // Store detected colors and their positions
        private List<String> detectedColors = new ArrayList<>();
        private List<Point> detectedPositions = new ArrayList<>();
        private List<String> lastDetectedColors = new ArrayList<>();
        private List<Point> lastDetectedPositions = new ArrayList<>();
        String colorType;
        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            // Initialize Mats with the appropriate size and type if needed
            maskRed = Mat.zeros(width, height, CvType.CV_8UC1);
            maskBlue = Mat.zeros(width, height, CvType.CV_8UC1);
            maskYellow = Mat.zeros(width, height, CvType.CV_8UC1);
            mask = Mat.zeros(width, height, CvType.CV_8UC1);
            hierarchy = Mat.zeros(width, height, CvType.CV_8UC1);
        }

        @Override
        public Mat processFrame(Mat frame, long captureTimeNanos) {
            return frame;
        }

        // Implementing the required processFrame method from OpenCvPipeline
        @Override
        public Mat processFrame(Mat input) {
            {
                if (input != null && !input.empty()) {
                    // Process frame
                    // Convert the input frame from BGR to HSV color space
                    Imgproc.cvtColor(input, hsvImage, Imgproc.COLOR_BGR2HSV);

                    // Clear previous contours and masks
                    // mask.release();
                    contours.clear();
                    detectedColors.clear();
                    detectedPositions.clear();
                    lastDetectedColors.clear();
                    lastDetectedPositions.clear();

                    // Initialize masks to the same size as the input frame
                    maskRed = Mat.zeros(input.size(), input.type());
                    maskBlue = Mat.zeros(input.size(), input.type());
                    maskYellow = Mat.zeros(input.size(), input.type());
                    mask = Mat.zeros(input.size(), input.type());  // To store combined masks

                    Core.inRange(hsvImage, LOWER_RED_1, UPPER_RED_1, maskRed);
                    Core.inRange(hsvImage, LOWER_BLUE, UPPER_BLUE, maskBlue);
                    Core.inRange(hsvImage, LOWER_YELLOW, UPPER_YELLOW, maskYellow);

                    // Combine all masks
                    Core.bitwise_or(maskRed, maskBlue, mask);
                    Core.bitwise_or(mask, maskYellow, mask);
                    // Find contours
                    Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

                    int contourCount = Math.min(contours.size(), maxContours);
                    // Sort contours by area (from largest to smallest)
                    // contours.sort((contour1, contour2) -> Double.compare(Imgproc.contourArea(contour2), Imgproc.contourArea(contour1)));
                    // Loop through each contour
                    for (int i = 0; i < contourCount; i++) {
                        MatOfPoint contour = contours.get(i);
                        Rect rect = Imgproc.boundingRect(contour);
                        if (rect.width > 20 && rect.height > 10) { // Adjust area threshold
                            Imgproc.rectangle(input, rect.tl(), rect.br(), new Scalar(180, 0, 45), 3);

                            // Detect color in the region of interest
                            Mat roi = hsvImage.submat(rect);
                            ColorResult result = detectColor(roi);
                            roi.release(); // Release the submat after use

                            String colorType = result.colorType;
                            Scalar meanColor = result.meanColor;

                            // Store the mean color for all detections
                            lastDetectedColors.add(colorType);
                            lastDetectedPositions.add(new Point(rect.x + rect.width / 2, rect.y)); // Center the text
                            // Store the current scalar value for display
                            if (meanColor != null) {
                                detectedColors.add(String.format("R: %.0f, G: %.0f, B: %.0f", meanColor.val[0], meanColor.val[1], meanColor.val[2]));
                            } else {
                                detectedColors.add("N/A"); // Fallback if color can't be calculated
                            }
                            // If fewer detections than the max allowed, fill remaining detectedColors with "N/A"
                            while (detectedColors.size() < lastDetectedColors.size()) {
                                detectedColors.add("N/A");
                            }
                        }
                    }
                }
                return input; // Return the processed frame
            }
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
            Paint paint = new Paint();
            paint.setColor(Color.WHITE);
            paint.setTextSize(25 * scaleCanvasDensity);

            // Avoid flickering by displaying only last known colors
            for (int i = 0; i < lastDetectedColors.size(); i++) {
                // Ensure we have a corresponding position for each color
                if (i < lastDetectedPositions.size()) {
                    String colorText = lastDetectedColors.get(i);
                    Point position = lastDetectedPositions.get(i);

                    // Draw color text
                    float colorTextX = (float) position.x * scaleBmpPxToCanvasPx;
                    float colorTextY = (float) position.y * scaleBmpPxToCanvasPx - 10 * scaleCanvasDensity;
                    canvas.drawText(colorText, colorTextX, colorTextY, paint);

                    // Draw the current Scalar value below the color text with a fixed offset
                    if (i < detectedColors.size()) {
                        String scalarText = detectedColors.get(i);
                        float scalarTextY = colorTextY + 40 * scaleCanvasDensity; // Adjust the offset as needed
                        canvas.drawText(scalarText, colorTextX, scalarTextY, paint);
                    }
                } else {

                    // Log or handle cases where position data is missing
                    System.err.println("No position found for color: " + lastDetectedColors.get(i));
                }
            }
        }


        private ColorResult detectColor(Mat roi) {
                // Calculate the average color in the region of interest (ROI)
                Imgproc.cvtColor(roi, hsvRoi, Imgproc.COLOR_BGR2HSV);
                Scalar meanColor = Core.mean(hsvRoi);

                // Check which color the mean color belongs to
                colorType = "Unknown";
                if (isColorInRange(meanColor, LOWER_RED_1, UPPER_RED_1)) {
                    colorType = "Red";
                } else if (isColorInRange(meanColor, LOWER_BLUE, UPPER_BLUE)) {
                    colorType = "Blue";
                } else if (isColorInRange(meanColor, LOWER_YELLOW, UPPER_YELLOW)) {
                    colorType = "Yellow";
                }
                if (hsvRoi != null) hsvRoi.release();

                return new ColorResult(colorType, meanColor);
            }

        private boolean isColorInRange(Scalar color, Scalar lower, Scalar upper) {
            return color.val[0] >= lower.val[0] && color.val[0] <= upper.val[0] &&
                    color.val[1] >= lower.val[1] && color.val[1] <= upper.val[1] &&
                    color.val[2] >= lower.val[2] && color.val[2] <= upper.val[2];
        }
    }
