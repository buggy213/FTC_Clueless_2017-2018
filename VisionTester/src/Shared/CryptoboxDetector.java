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

/**
 * Created by Joshua on 9/25/2017.
 */

public class CryptoboxDetector {

    static Cryptobox cryptobox;

    final static int minLineLength = 50;

    final static double minArea = 500;
    final static double maxArea = 4001;

    final static double maxGap = 35;

    static Scalar lowerRed = new Scalar(150, 0, 0);
    static Scalar upperRed = new Scalar(255, 120, 120);

    static Mat redArea = new Mat();

    static Mat rgb = new Mat();

    static Mat blurredImg = new Mat();

    static Mat image = new Mat();

    static Mat thresholdImage = new Mat();

    static Mat hierarchy = new Mat();

    static Mat debugImage = new Mat();

    static List<Contour> contours = new ArrayList<>();

    // public static Cryptobox getCryptobox() {
    //     return cryptobox;
    // }

    public static Mat detectCryptobox(Mat rgba, Mat gray) {
        // if (cryptobox == null) {
        //     cryptobox = new Cryptobox();
        // }
        debugImage = rgba.clone();
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



        Imgproc.cvtColor(rgba, rgb, Imgproc.COLOR_BGR2RGB);

        Core.inRange(rgb, lowerRed, upperRed, redArea);

        //Dilate (blur) the mask to decrease processing power
        Imgproc.dilate(redArea, blurredImg, new Mat());

        List<MatOfPoint> contourListTemp = new ArrayList<>();

        Imgproc.findContours(blurredImg, contourListTemp, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Filter contours by area and resize to fit the original image size
        contours.clear();
        for (MatOfPoint c : contourListTemp) {
            contours.add(new Contour(c));
        }

        List<Contour> removalList = new ArrayList<>();
        for (Contour c : contours) {
            if (c.area() < minArea || c.area() > maxArea) {
                removalList.add(c);
            }
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
        }
        // At this point we should only have contours representing the walls of the cryptobox


        Imgcodecs.imwrite("imgs/red.jpg", redArea);

        Imgcodecs.imwrite("imgs/debug.jpg", debugImage);

        return rgba;

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
