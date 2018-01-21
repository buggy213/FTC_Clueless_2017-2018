package org.firstinspires.ftc.teamcode.Shared.VisionProcessing;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

/**
 * Created by Joshua on 9/25/2017.
 */

public class CryptoboxDetector {

    static Cryptobox cryptobox;

    final static int lowThreshold = 10;
    final static int highThreshold = 100;

    final static int minLineLength = 150;

    public static Cryptobox getCryptobox() {
        return cryptobox;
    }

    public static void detectCryptobox(Mat rgba, Mat gray) {
        if (cryptobox == null) {
            cryptobox = new Cryptobox();
        }

        Mat image = gray.clone();
        Mat thresholdImage = gray.clone();
        Imgproc.blur(image, thresholdImage, new Size(2, 2));
        Imgproc.Canny(thresholdImage, thresholdImage, 5, 75, 3, true);

        Mat lines = new Mat();
        int threshold = 50;
        int minLineSize = 20;
        int lineGap = 20;

        Imgproc.HoughLinesP(thresholdImage, lines, 1, Math.PI / 180, threshold, minLineSize, lineGap);

        ArrayList<Line> lineArrayList = new ArrayList<>();

        for (int x = 0; x < lines.cols(); x++)
        {
            double[] vec = lines.get(0, x);
            double x1 = vec[0],
                    y1 = vec[1],
                    x2 = vec[2],
                    y2 = vec[3];
            Point start = new Point(x1, y1);
            Point end = new Point(x2, y2);

            lineArrayList.add(new Line(start, end));

            Drawing.drawLine(rgba, start, end, new Scalar(255, 0, 255), 3);
        }

        lineArrayList = cullShortLines(lineArrayList, minLineLength);

        ArrayList<Line> verticalLines = new ArrayList<>();

        // At this point we should have long lines
        for (Line l: lineArrayList) {
            if (l.vertical(15)) {
                verticalLines.add(l);
            }
        }

        

        // At this point we should have vertical lines


    }


    static ArrayList<Line> cullShortLines(ArrayList<Line> lines, double minLength) {
        ArrayList<Line> lineList = new ArrayList<>();
        for(Line l : lines) {
            if (l.length() > minLength)
                lineList.add(l);
        }

        return lineList;
    }

}
