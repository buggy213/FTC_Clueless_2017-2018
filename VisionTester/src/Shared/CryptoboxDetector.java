package Shared;

import Shared.vision.*;
import org.opencv.core.*;
import org.opencv.features2d.FeatureDetector;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Queue;

import static org.opencv.core.Core.flip;
import static org.opencv.core.Core.mean;
import static org.opencv.core.Core.transpose;
import static org.opencv.core.CvType.CV_8U;
import static org.opencv.core.CvType.CV_8UC1;

public class CryptoboxDetector {

    public boolean passThrough = true;

    public Size frameSize = new Size(640, 360); // Default is (1/2)^2 of 1280x720

    public class CryptoState {
        LinkedHashMap<Integer, ColumnState> columnStates = new LinkedHashMap<>();
        int index = 0;
    }

    public class ColumnState {
        public double lastKnownPosition;
        public int framesSinceVisible;
        public int id;
        final int tolerance = 50;
        @Override
        public boolean equals(Object o) {
            if (!(o instanceof ColumnState))
                return false;
            return Math.abs(((ColumnState)o).lastKnownPosition - lastKnownPosition) < tolerance;
        }

        public ColumnState (double initialPosition, int id) {
            lastKnownPosition = initialPosition;
            this.id = id;
        }
    }

    public double getCenterPoint(int id1, int id2) {
        if (state.columnStates.containsKey(id1) && state.columnStates.containsKey(id2)) {
            Double mid = (state.columnStates.get(id1).lastKnownPosition + state.columnStates.get(id2).lastKnownPosition) / 2;
            return mid;
        }
        else {
            return -1;
        }
    }

    Comparator<Contour> leftToRight = new Comparator<Contour>() {
        @Override
        public int compare(Contour contour, Contour t1) {
            return (int) (contour.center().x - t1.center().x);
        }
    };

    Comparator<Contour> rightToLeft = new Comparator<Contour>() {
        @Override
        public int compare(Contour contour, Contour t1) {
            return -1 * (int) (contour.center().x - t1.center().x);
        }
    };

    Comparator<Contour> area = new Comparator<Contour>() {
        @Override
        public int compare(Contour contour, Contour t1) {
            return (int) (contour.area() - t1.area());
        }
    };

    List<Contour> previousFrameContours;

    CryptoState state = new CryptoState();

    public CryptoState getCurrentState() {
        return state;
    }

    final double sizeFactor = 0.5;
    final double minHeightWidthRatio = 1.2;
    final double testGapAreaFactor = 0.8;
    final int horizontalDistanceCenterThreshold = 60;
    final int horizontalThreshold = 60;

    final double maxFramesNotVisible = 15;


    DogeCVColorFilter colorFilter;

    Mat debugImage = new Mat();
    Mat workingMat = new Mat();
    Mat mask = new Mat();
    Mat hierarchy = new Mat();
    Mat rot = new Mat();

    final int minArea = 400;

    List<Contour> previousFrameBiggestContours;

    TeamColor color;
    Direction direction;

    boolean firstTime;

    public Mat processFrame(Mat rgba, Mat gray) {
        if (passThrough) {
            return rgba;
        }
        detectCryptobox(rgba, gray);
        /*if (!firstTime) {
            if (color == TeamColor.BLUE) {
                try {
                    AppUtil.getInstance().getActivity().setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);
                    Thread.sleep(1000);
                    AppUtil.getInstance().getActivity().setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_PORTRAIT);
                    Thread.sleep(1000);
                    AppUtil.getInstance().getActivity().setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);
                } catch (InterruptedException e) {
                    // Swallow
                }
            }
            else {
                try {
                    AppUtil.getInstance().getActivity().setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_REVERSE_LANDSCAPE);
                    Thread.sleep(1000);
                    AppUtil.getInstance().getActivity().setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_PORTRAIT);
                    Thread.sleep(1000);
                    AppUtil.getInstance().getActivity().setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_REVERSE_LANDSCAPE);
                } catch (InterruptedException e) {
                    // Swallow
                }
            }
            firstTime = true;
        }*/

        return debugImage;
    }

    public void init(TeamColor color) {
        this.color = color;
        if (color == TeamColor.BLUE) {
            direction = Direction.LEFT;
            colorFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.BLUE);
        } else {
            direction = Direction.RIGHT;
            colorFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.RED);
        }
    }

    public void detectCryptobox(Mat rgb, Mat gray) {
        rgb.copyTo(workingMat);
        Size initialSize = rgb.size();
        Size newSize = new Size(initialSize.width * sizeFactor, initialSize.height * sizeFactor);
        frameSize = newSize;
        if(color == TeamColor.RED){
            Core.flip(workingMat, workingMat, -1); //mRgba.t() is the transpose
        }

        Imgproc.resize(workingMat, workingMat, newSize);

        workingMat.copyTo(debugImage);

        Imgproc.blur(workingMat, workingMat, new Size(6, 6));

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
            } else {
                Drawing.drawContour(debugImage, c, Color.create(BasicColors.GREEN, ColorSpace.RGB));
            }
        }

        contours.removeAll(degenerateContours);
        // Coming from left
        if (direction == Direction.LEFT) {
            Collections.sort(contours, leftToRight);
        } else {
            Collections.sort(contours, rightToLeft);
        }
        List<List<Contour>> contourGroups = new ArrayList<>();
        double currentAverage = 0;
        int groupIndex = 0;
        if (contours.size() > 0) {
            contourGroups.add(new ArrayList<Contour>());
            contourGroups.get(0).add(contours.get(0));
            currentAverage = contours.get(0).center().x;
        }
        for (int i = 1; i < contours.size(); i++) {
            if (Math.abs(currentAverage - contours.get(i).center().x) < horizontalThreshold) {
                contourGroups.get(groupIndex).add(contours.get(i));
                double sum = 0;
                for (Contour c : contourGroups.get(groupIndex)) {
                    sum += c.center().x;
                }
                currentAverage = sum / contourGroups.get(groupIndex).size();
            }
            else {
                contourGroups.add(new ArrayList<Contour>());
                groupIndex++;
                contourGroups.get(groupIndex).add(contours.get(i));
                currentAverage = contours.get(i).center().x;
            }

        }

        List<Contour> biggestContours = new ArrayList<>();
        for (int i = 0; i < contourGroups.size(); i++) {
            Collections.sort(contourGroups.get(i), area);
            biggestContours.add(contourGroups.get(i).get(contourGroups.get(i).size() - 1));
            Drawing.drawContour(debugImage, biggestContours.get(i), Color.create(BasicColors.ORANGE, ColorSpace.RGB));
        }

        if (previousFrameContours == null) {
            previousFrameContours = new ArrayList<>();
        }

        /*for (Contour c : previousFrameContours) {
            List<Double> deltaXVals = new ArrayList<>();
            for (Contour other : biggestContours) {
                deltaXVals.add(c.center().x - other.center().x);
            }
            Collections.sort(deltaXVals);
            if (deltaXVals.size() >= 1) {
                if (deltaXVals.get(deltaXVals.size() - 1) > maxDelta) {
                    // Contour has fallen off the screen
                    state.index++;
                }
            }
        }*/

        List<Double> columnXVals = new ArrayList<>();
        for (Contour c : biggestContours) {
            //  Drawing.drawText(debugImage, String.valueOf(index + state.index), c.center(), 0.5f, Color.create(BasicColors.WHITE));
            columnXVals.add(c.center().x);
        }
        List<Double> midXVals = new ArrayList<>();

        for (int i = 0; i < biggestContours.size() - 1; i++) {
            Drawing.drawCircle(debugImage, average(biggestContours.get(i).center(), biggestContours.get(i + 1).center()), 4, Color.create(BasicColors.ORANGE, ColorSpace.RGB));
            midXVals.add(average(biggestContours.get(i).center(),(biggestContours.get(i + 1).center())).x);
        }
        List<Integer> removalList = new ArrayList<>();
        // Update to find new positions of tracked objects
        for (ColumnState columnState : state.columnStates.values()) {
            boolean visible = false;

            for (Double xVal : columnXVals) {
                if (Math.abs(xVal-columnState.lastKnownPosition) < columnState.tolerance) {
                    columnState.lastKnownPosition = xVal;
                    visible = true;
                    columnState.framesSinceVisible = 0;
                }
            }

            if (!visible) {
                columnState.framesSinceVisible++;
            }

            if (columnState.framesSinceVisible > maxFramesNotVisible) {
                removalList.add(columnState.id);
            }
        }

        for (int i: removalList) {
            state.columnStates.remove(i);
        }
        // Detect if any major contours don't have a column associated with them
        for (Double xVal : columnXVals) {
            boolean hasAColumn = false;
            for (ColumnState columnState : state.columnStates.values()) {
                if (Math.abs(columnState.lastKnownPosition - xVal) < columnState.tolerance) {
                    hasAColumn = true;
                }
            }

            if (!hasAColumn) {
                state.columnStates.put(state.index++, new ColumnState(xVal, state.index - 1));
            }
        }
        Iterator iterator = state.columnStates.entrySet().iterator();
        while (iterator.hasNext()) {
            Map.Entry pair = (Map.Entry)iterator.next();
            ColumnState columnState = (ColumnState)(pair.getValue());
            Drawing.drawText(debugImage, String.valueOf((int)pair.getKey()), new Point(columnState.lastKnownPosition, 180), 0.5f, Color.create(BasicColors.GREEN, ColorSpace.RGB));
        }
        Imgproc.cvtColor(debugImage, debugImage, Imgproc.COLOR_BGR2RGB);
        Imgcodecs.imwrite("imgs/output.jpg",debugImage);
        previousFrameContours = biggestContours;
        Imgproc.resize(debugImage, debugImage, initialSize);
    }
    Point average (Point p1, Point p2) {
        return new Point((p1.x + p2.x) / 2, (p1.y + p2.y) / 2);
    }
    Rectangle fromRect(Rect rect) {
        return new Rectangle(rect.x, rect.y, rect.x + rect.width, rect.y + rect.height);
    }
}
