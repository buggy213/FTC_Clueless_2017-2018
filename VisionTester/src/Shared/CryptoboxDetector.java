package Shared;

import Shared.vision.*;
import org.opencv.core.*;
import org.opencv.features2d.FeatureDetector;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Comparator;
import java.util.List;

import static org.opencv.core.CvType.CV_8U;
import static org.opencv.core.CvType.CV_8UC1;

/**
 * Created by Joshua on 9/25/2017.
 */

public class CryptoboxDetector {


    final static int minLineLength = 50;

    final static double minArea = 500;
    final static double maxArea = 4001;

    final static double maxGap = 35;

    final static double minHeightRatio = 1.2;

    static Scalar lowerRed = new Scalar(10, 255, 255);
    static Scalar upperRed = new Scalar(180, 255, 255);

    static Mat redArea = new Mat();

    static Mat blurredImg = new Mat();

    static Mat image = new Mat();

    static Mat thresholdImage = new Mat();

    static Mat hierarchy = new Mat();

    static Mat debugImage = new Mat();


    static Mat mask1 = new Mat();
    static Mat mask2 = new Mat();

    static List<Contour> contours = new ArrayList<>();

    public static List<List<Contour>> detectCryptobox(Mat rgb, Mat gray) {
        debugImage = rgb.clone();
        image = gray.clone();
        thresholdImage = gray.clone();

        Imgproc.blur(image, thresholdImage, new Size(2, 2));
        Imgproc.Canny(thresholdImage, thresholdImage, 5, 75, 3, true);

        Imgcodecs.imwrite("imgs/canny.jpg", thresholdImage);

        Mat lines = new Mat();

        Imgproc.HoughLinesP(thresholdImage, lines, 1, Math.PI / 180, 100, 40, 5);

        System.out.println("Number of lines found: " + lines.rows());

        ArrayList<Line> lineArrayList = new ArrayList<>();

        for (int x = 0; x < lines.rows(); x++)
        {
            double[] vec = lines.get(x, 0);
            double x1 = vec[0],
                    y1 = vec[1],
                    x2 = vec[2],
                    y2 = vec[3];
            Point start = new Point(x1, y1);
            Point end = new Point(x2, y2);

            lineArrayList.add(new Line(start, end));
            // ineArrayList.get(x).draw(rgba, new Scalar(0, 255,255), 2);
        }

        lineArrayList = cullShortLines(lineArrayList, minLineLength);

        ArrayList<Line> verticalLines = new ArrayList<>();

        // At this point we should have long lines
        for (Line l: lineArrayList) {
            if (l.vertical(15)) {
                verticalLines.add(l);
            }
        }

        System.out.println("Vertical lines found: " + verticalLines.size());

        for (Line l: verticalLines) {
            // l.draw(debugImage, new Scalar(255, 255, 0), 2);
        }

        Mat kernel = Mat.ones(5,5, CV_8UC1);

        Imgproc.cvtColor(rgb, blurredImg, Imgproc.COLOR_BGR2HSV);

        Imgproc.erode(blurredImg, blurredImg, kernel);
        // Dilate (blur) the mask to decrease processing power
        Imgproc.dilate(blurredImg, blurredImg, kernel);
        Imgproc.blur(blurredImg, blurredImg, new Size(6,6));

        Mat blurRgb = new Mat();
        Imgproc.cvtColor(blurredImg, blurRgb, Imgproc.COLOR_HSV2BGR);
        Imgcodecs.imwrite("imgs/blurredimg.jpg", blurRgb);
        blurRgb.release();


        Core.inRange(blurredImg, new Scalar(0, 100, 100), lowerRed, mask1);
        Core.inRange(blurredImg, new Scalar(170, 100, 100), upperRed, mask2);

        Core.bitwise_or(mask1, mask2, redArea);

        // Mat structure = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(40, 100));
        // Mat morphedImage = new Mat();
        // Imgproc.morphologyEx(blurredImg, morphedImage, Imgproc.MORPH_CLOSE, structure);

        List<MatOfPoint> contourListTemp = new ArrayList<>();

        Imgproc.findContours(redArea, contourListTemp, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Filter contours by area and resize to fit the original image size
        contours.clear();
        for (MatOfPoint c : contourListTemp) {
            contours.add(new Contour(c));
        }

        List<Contour> removalList = new ArrayList<>();
        int indexOfMatOfPoints = 0;
        for (Contour c : contours) {
            if (c.area() < minArea || c.area() > maxArea) {
                removalList.add(c);
                continue;
            }

            Rect rect = Imgproc.boundingRect(contourListTemp.get(indexOfMatOfPoints));
            if (rect.height / rect.width < minHeightRatio) {
                removalList.add(c);
            }
            indexOfMatOfPoints++;
        }

        contours.removeAll(removalList);

        for (Contour c : contours) {
            Drawing.drawContour(debugImage, c, Color.create(new Scalar(0,255,0), ColorSpace.RGB));
        }

        // Sort contours by how far right they are
        contours.sort(((o1, o2) -> {return (int)(o1.center().x - o2.center().x);}));

        List<List<Contour>> contourGroups = new ArrayList<>();

        for (int i = 0; i < contours.size(); i++) {
            if (i == contours.size() - 1) {
                break;
            }

            if (Math.abs(contours.get(i).center().x - contours.get(i + 1).center().x) < maxGap) {
                // Part of the same group
                if (contourGroups.size() == 0) {
                    // First group
                    contourGroups.add(new ArrayList<>());
                    contourGroups.get(0).add(contours.get(i));
                    contourGroups.get(0).add(contours.get(i + 1));
                }

                else {
                    contourGroups.get(contourGroups.size() - 1).add(contours.get(i + 1));
                }
            }
            else {
                // New group
                contourGroups.add(new ArrayList<>());
                contourGroups.get(contourGroups.size() - 1).add(contours.get(i + 1));
            }
        }

        int index = 0;
        for (List<Contour> contourGroup : contourGroups) {
            System.out.println("Contour group: " + index);

            for (Contour c : contourGroup) {
                System.out.println(c.center().x);
            }

            index++;
        }

        // We now have groups of contours to work with. Hooray!


        Imgcodecs.imwrite("imgs/red.jpg", redArea);

        Imgcodecs.imwrite("imgs/debug.jpg", debugImage);

        return contourGroups;
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
