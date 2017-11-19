package Shared;

import Shared.vision.Contour;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.util.List;

import static org.opencv.core.CvType.CV_8UC1;

public class GlyphDetector {
    public class Glyph {
        public GlyphColor color;
        public Contour outline;
        public Point center;
    }

    static Scalar lowerBrown = new Scalar(0, 25, 115);
    static Scalar upperBrown = new Scalar(23, 204, 255);

    public static List<Glyph> detectGlyphs(Mat rgb, Mat grey) {
        // First find brown glyphs
        Mat image = rgb.clone();
        Mat thresholdImage = new Mat();

        Mat brown = new Mat();

        Mat kernel = Mat.ones(5,5, CV_8UC1);

        Imgproc.erode(image, image, kernel);
        // Dilate (blur) the mask to decrease processing power
        Imgproc.dilate(image, image, kernel);
        Imgproc.blur(image, image, new Size(6,6));

        Imgproc.cvtColor(image, image, Imgproc.COLOR_BGR2HSV);

        Core.inRange(image, lowerBrown, upperBrown, brown);

        Imgcodecs.imwrite("imgs/brown.png", brown);

        brown.release();
        image.release();
        thresholdImage.release();

        return null;
    }
}
