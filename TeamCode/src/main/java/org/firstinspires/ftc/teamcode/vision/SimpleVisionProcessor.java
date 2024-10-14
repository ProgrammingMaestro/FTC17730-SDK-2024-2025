package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Typeface;
//import android.text.TextPaint;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.Locale;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;

public class SimpleVisionProcessor implements VisionProcessor {
    private Paint textPaint;
    private Paint linePaint;
    private final Mat hierarchy = new Mat();
    private final Mat sel1 = new Mat(); // these facilitate capturing through 0
    private final Mat sel2 = new Mat();
    private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));
    private double largestContourX;
    private double largestContourY;
    private double largestContourArea;
    private MatOfPoint largestContour;
    private PropPositions previousPropPosition;
    private PropPositions recordedPropPosition = PropPositions.UNFOUND;

    /*
    public SimpleVisionProcessor(@NonNull Scalar lower, @NonNull Scalar upper, DoubleSupplier minArea, DoubleSupplier left, DoubleSupplier right) {
        this.contours = new ArrayList<>();
        this.lower = lower;
        this.upper = upper;
        this.minArea = minArea;
        this.left = left;
        this.right = right;

        // setting up the paint for the text in the center of the box

        textPaint = new Paint();
        textPaint.setColor(Color.GREEN);
        textPaint.setAntiAlias(true);
        textPaint.setTextSize(40); // or this
        textPaint.setTypeface(Typeface.DEFAULT_BOLD);

        // setting up the paint for the lines that comprise the box
        linePaint = new Paint();
        linePaint.setColor(Color.GREEN); // you may want to change this
        linePaint.setAntiAlias(true);
        linePaint.setStrokeWidth(10); // or this
        linePaint.setStrokeCap(Paint.Cap.ROUND);
        linePaint.setStrokeJoin(Paint.Join.ROUND);
    }
*/
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        // this method comes with all VisionProcessors, we just don't need to do anything here, and you dont need to call it
    }

    /**
     * @return the x position of the currently found largest contour in the range [0, camera width], or -1 if no largest contour has been determined
     */
    public double getLargestContourX() {
        return largestContourX;
    }

    /**
     * @return the y position of the currently found largest contour in the range [0, camera height], or -1 if no largest contour has been determined
     */
    public double getLargestContourY() {
        return largestContourY;
    }

    /**
     * @return the area of the currently found largest contour, or -1 if no largest contour has been determined
     */
    public double getLargestContourArea() {
        return largestContourArea;
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        // this method processes the image (frame) taken by the camera, and tries to find a suitable prop
        // you dont need to call it

        // this converts the frame from RGB to HSV, which is supposed to be better for doing colour blob detection
        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGB2HSV);

        // sets up the center points of our largest contour to be -1 (offscreen)
        largestContourX = largestContourY = -1;

        // if we found it, calculates the actual centers
        if (largestContour != null) {
            Moments moment = Imgproc.moments(largestContour);
            largestContourX = (moment.m10 / moment.m00);
            largestContourY = (moment.m01 / moment.m00);
        }

        // if we have found a new prop position, and it is not unfound, updates the recorded position,
        // this makes sure that if our camera is playing up, we only need to see the prop in the correct position
        // and we will hold onto it


//		Imgproc.drawContours(frame, contours, -1, colour);

        // returns back the edited image, don't worry about this too much
        Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(frame, b);
        lastFrame.set(b);
        return frame;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // this method draws the rectangle around the largest contour and puts the current prop position into that rectangle
        // you don't need to call it

//		for (MatOfPoint contour : contours) {
//			Rect rect = Imgproc.boundingRect(contour);
//			canvas.drawLines(new float[]{rect.x * scaleBmpPxToCanvasPx, rect.y * scaleBmpPxToCanvasPx, (rect.x + rect.width) * scaleBmpPxToCanvasPx, (rect.y + rect.height) * scaleBmpPxToCanvasPx}, textPaint);
//		}

        // if the contour exists, draw a rectangle around it and put its position in the middle of the rectangle
        textPaint = new Paint();
        textPaint.setColor(Color.GREEN);
        textPaint.setAntiAlias(true);
        textPaint.setTextSize(40); // or this
        textPaint.setTypeface(Typeface.DEFAULT_BOLD);

        // setting up the paint for the lines that comprise the box
        linePaint = new Paint();
        linePaint.setColor(Color.GREEN); // you may want to change this
        linePaint.setAntiAlias(true);
        linePaint.setStrokeWidth(10); // or this
        linePaint.setStrokeCap(Paint.Cap.ROUND);
        linePaint.setStrokeJoin(Paint.Join.ROUND);
        if (largestContour != null) {
            Rect rect = Imgproc.boundingRect(largestContour);
            float[] points = {rect.x * scaleBmpPxToCanvasPx, rect.y * scaleBmpPxToCanvasPx, (rect.x + rect.width) * scaleBmpPxToCanvasPx, (rect.y + rect.height) * scaleBmpPxToCanvasPx};

            canvas.drawLine(points[0], points[1], points[0], points[3], linePaint);
            canvas.drawLine(points[0], points[1], points[2], points[1], linePaint);

            canvas.drawLine(points[0], points[3], points[2], points[3], linePaint);
            canvas.drawLine(points[2], points[1], points[2], points[3], linePaint);

            String text = String.format(Locale.ENGLISH, "%s", recordedPropPosition.toString());

            canvas.drawText(text, (float) largestContourX * scaleBmpPxToCanvasPx, (float) largestContourY * scaleBmpPxToCanvasPx, textPaint);
        }
    }

    /**
     * @return the last found prop position, if none have been found, returns {@link PropPositions#UNFOUND}
     */
    public PropPositions getRecordedPropPosition() {
        return recordedPropPosition;
    }

    // returns the largest contour if you want to get information about it
    public MatOfPoint getLargestContour() {
        return largestContour;
    }

    /*
    @Override
    protected void finalize() throws Throwable {
        close();
        super.finalize();
    }
*/
    public void close() {
        hierarchy.release();
        sel1.release();
        sel2.release();
    }

    // the enum that stores the 4 possible prop positions
    public enum PropPositions {
        LEFT,
        MIDDLE,
        RIGHT,
        UNFOUND;
    }
}