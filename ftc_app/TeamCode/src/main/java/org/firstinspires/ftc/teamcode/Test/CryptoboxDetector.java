package org.firstinspires.ftc.teamcode.Test;

import android.content.Context;

import com.disnodeteam.dogecv.OpenCVPipeline;
import com.disnodeteam.dogecv.ViewDisplay;
import com.disnodeteam.dogecv.filters.DogeCVColorFilter;
import com.disnodeteam.dogecv.filters.LeviColorFilter;

import org.firstinspires.ftc.teamcode.Test.vision.*;
import org.opencv.core.*;
import org.opencv.features2d.FeatureDetector;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Queue;

import static org.opencv.core.Core.mean;
import static org.opencv.core.CvType.CV_8U;
import static org.opencv.core.CvType.CV_8UC1;

public class CryptoboxDetector extends OpenCVPipeline{

    public class CryptoState {
        public Queue<Contour> currentlyVisible;
        public int leftIndex;
    }

    CryptoState state = new CryptoState();

    final double sizeFactor = 0.5;
    final double minHeightWidthRatio = 1.2;
    final double testGapAreaFactor = 0.8;
    final int horizontalDistanceCenterThreshold = 15;
    DogeCVColorFilter colorFilter;
    
    Mat debugImage = new Mat();
    Mat workingMat = new Mat();
    Mat mask = new Mat();
    Mat hierarchy = new Mat();

    final int minArea = 250;

    TeamColor color;
    
    @Override
    public Mat processFrame(Mat rgba, Mat gray) {
        detectCryptobox(rgba, gray);
        return debugImage;
    }

    public void init(Context context, final ViewDisplay viewDisplay, final int cameraIndex, TeamColor color) {
        this.color = color;
        colorFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.RED);
        super.init(context, viewDisplay, cameraIndex);
    }

    public void detectCryptobox(Mat rgb, Mat gray) {
        rgb.copyTo(workingMat);
        Size initialSize = rgb.size();
        Size newSize = new Size(initialSize.width * sizeFactor, initialSize.height * sizeFactor);
        Imgproc.resize(workingMat, workingMat, newSize);

        workingMat.copyTo(debugImage);

        Imgproc.blur(workingMat, workingMat, new Size(6,6));
        
        List<MatOfPoint> contourListTemp = new ArrayList<>();
        Mat convert = workingMat.clone();
        colorFilter.process(convert, mask);
        convert.release();
        Imgproc.findContours(mask, contourListTemp, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        List<Contour> contours = new ArrayList<>();
        for (MatOfPoint points : contourListTemp) {
            contours.add(new Contour(points));
        }

        List<Contour> degenerateContours = new ArrayList<>();

        for (Contour c : contours) {
            String status = "";
            double score = 0;

            if (c.area() < minArea) {
                status += "S";
                score -= 2;
            }
            if (c.height() / c.width() < minHeightWidthRatio) {
                status += "W";
                score -= 2;
            }

            for (Contour other : contours) {
                if (Math.abs(other.center().x - c.center().x) < horizontalDistanceCenterThreshold && other.area() > minArea && c.area() > minArea) {
                    status += "G";
                    score += 1;
                    /*Contour top = other.center().y > c.center().y ? other : c;
                    Contour bottom = top == c ? other : c;
                    Rect ROI = new Rect(top.bottomLeft(), bottom.topRight());
                    Mat area = workingMat.submat(ROI);
                    Imgproc.cvtColor(area, area, Imgproc.COLOR_RGB2HLS);
                    Mat grayArea = new Mat();
                    Core.inRange(area, new Scalar(0, 0, 0), new Scalar(0, 255, 35), grayArea);
                    if (Core.mean(grayArea).val[0] > testGapAreaFactor * 255) {
                        score += 1;
                        status += "W";
                        Drawing.drawRectangle(debugImage, fromRect(ROI), Color.create(BasicColors.GREEN, ColorSpace.RGB));
                    }
                    else {
                        Drawing.drawRectangle(debugImage, fromRect(ROI), Color.create(BasicColors.RED, ColorSpace.RGB));
                    }

                    area.release();
                    grayArea.release();
                    */
                }
            }
            Drawing.drawText(debugImage, status + "," + score, c.center(), 0.5f, Color.create(BasicColors.WHITE, ColorSpace.RGB));
            if (score < 0) {
                degenerateContours.add(c);
                Drawing.drawContour(debugImage, c, Color.create(BasicColors.RED, ColorSpace.RGB));
            }
            else {
                Drawing.drawContour(debugImage, c, Color.create(BasicColors.GREEN, ColorSpace.RGB));
            }
        }

        contours.removeAll(degenerateContours);

        for (Contour c : contours) {
            
        }

        Imgproc.resize(debugImage, debugImage, initialSize);
    }

    Rectangle fromRect(Rect rect) {
        return new Rectangle(rect.x, rect.y, rect.x + rect.width, rect.y + rect.height);
    }
}
