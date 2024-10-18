package vision;

import org.opencv.core.Scalar;

public class ColorResult {
    public String colorType;
    public Scalar meanColor;

    public ColorResult(String colorType, Scalar meanColor) {
        this.colorType = colorType;
        this.meanColor = meanColor;
    }
}
