package org.firstinspires.ftc.teamcode.Test;

import android.content.Context;
import android.content.pm.ActivityInfo;

import com.disnodeteam.dogecv.OpenCVPipeline;
import com.disnodeteam.dogecv.ViewDisplay;
import com.disnodeteam.dogecv.filters.DogeCVColorFilter;
import com.disnodeteam.dogecv.filters.LeviColorFilter;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Shared.Direction;
import org.firstinspires.ftc.teamcode.Test.vision.*;
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
import java.util.List;
import java.util.Queue;

import static org.opencv.core.Core.flip;
import static org.opencv.core.Core.mean;
import static org.opencv.core.Core.transpose;
import static org.opencv.core.CvType.CV_8U;
import static org.opencv.core.CvType.CV_8UC1;

public class CryptoboxDetector extends OpenCVPipeline {

    public class CryptoState {
        HashMap<Integer, ColumnState> columnStates = new HashMap<>();
    }

    public class ColumnState {
        public int id;
        public double lastKnownPosition;
        public double deltaTimeSinceVisible = 0;
        public double nonvisibleStartTime;
        public boolean currentlyVisible = true;

        public ColumnState(double lastKnownPosition, int id) {
            this.lastKnownPosition = lastKnownPosition;
        }
    }

    public double getCenterPoint(int id1, int id2) {
        if (state.columnStates.containsKey(id1) && state.columnStates.containsKey(id2)) {
            return (state.columnStates.get(id1).lastKnownPosition + state.columnStates.get(id2).lastKnownPosition) / 2;
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
    final int horizontalDistanceCenterThreshold = 15;
    final int horizontalThreshold = 50;
    final int maxDelta = 40;

    final double maxTimeNotVisible = 250;
    ElapsedTime timer;

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

    @Override
    public Mat processFrame(Mat rgba, Mat gray) {
        detectCryptobox(rgba, gray);
        if (!firstTime) {
            try {
                AppUtil.getInstance().getActivity().setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);
                Thread.sleep(1000);
                AppUtil.getInstance().getActivity().setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_PORTRAIT);
                Thread.sleep(1000);
                AppUtil.getInstance().getActivity().setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);
            } catch (InterruptedException e) {
                // Swallow
            }
            firstTime = true;
        }

        return debugImage;
    }

    public void init(Context context, final ViewDisplay viewDisplay, final int cameraIndex, TeamColor color) {
        this.color = color;
        if (color == TeamColor.BLUE) {
            direction = Direction.LEFT;
        } else {
            direction = Direction.RIGHT;
        }
        colorFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.RED);
        timer = new ElapsedTime();
        super.init(context, viewDisplay, cameraIndex);
    }

    public void detectCryptobox(Mat rgb, Mat gray) {
        rgb.copyTo(workingMat);
        Size initialSize = rgb.size();
        Size newSize = new Size(initialSize.width * sizeFactor, initialSize.height * sizeFactor);
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

            // Drawing.drawText(debugImage, status + "," + score, c.center(), 0.5f, Color.create(BasicColors.WHITE, ColorSpace.RGB));
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
        for (int i = 0; i < contours.size(); i++) {
            if (i == contours.size() - 1) {
                break;
            }

            if (Math.abs(contours.get(i).center().x - contours.get(i + 1).center().x) < horizontalThreshold) {

                // Part of the same group
                if (contourGroups.size() == 0) {
                    // First group
                    contourGroups.add(new ArrayList<Contour>());
                    contourGroups.get(0).add(contours.get(i));
                    contourGroups.get(0).add(contours.get(i + 1));

                } else {
                    contourGroups.get(contourGroups.size() - 1).add(contours.get(i + 1));
                }
            } else {
                // New group
                contourGroups.add(new ArrayList<Contour>());
                contourGroups.get(contourGroups.size() - 1).add(contours.get(i + 1));
            }
        }

        List<Contour> biggestContours = new ArrayList<>();
        for (int i = 0; i < contourGroups.size(); i++) {
            Collections.sort(contourGroups.get(i), area);
            biggestContours.add(contourGroups.get(i).get(contourGroups.get(i).size() - 1));
            Drawing.drawContour(debugImage, biggestContours.get(i), Color.create(BasicColors.ORANGE));
        }

        if (previousFrameContours == null) {
            previousFrameContours = biggestContours;
            return;
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
            Drawing.drawCircle(debugImage, biggestContours.get(i).center().average(biggestContours.get(i + 1).center()), 4, Color.create(BasicColors.ORANGE));
            midXVals.add(biggestContours.get(i).center().average(biggestContours.get(i + 1).center()).x);
        }

        List<Double> accountedFor = new ArrayList<>();
        List<ColumnState> columnsAccountFor = new ArrayList<>();
        for (Double d : columnXVals) {
            for (ColumnState cs : state.columnStates.values()) {
                if (Math.abs(d - cs.lastKnownPosition) < horizontalThreshold) {
                    // Check for new columns
                    accountedFor.add(d);
                    columnsAccountFor.add(cs);
                    cs.currentlyVisible = true;
                    cs.deltaTimeSinceVisible = 0;
                    cs.lastKnownPosition = d;
                }
            }
        }

        List<Double> unaccountedFor = new ArrayList<>(midXVals);
        unaccountedFor.removeAll(accountedFor);

        if (direction == Direction.RIGHT) {
            Collections.sort(unaccountedFor, Collections.reverseOrder());
        }
        else {
            Collections.sort(unaccountedFor);
        }


        for (Double d : unaccountedFor) {
            // New columns!
            if (state.columnStates.size() == 0) {
                state.columnStates.put(0, new ColumnState(d, 0));
            }
            else {
                state.columnStates.put(Collections.max(state.columnStates.keySet()) + 1, new ColumnState(d, Collections.max(state.columnStates.keySet()) + 1));
            }
        }

        List<ColumnState> columnsUnaccountFor = new ArrayList<>(state.columnStates.values());
        columnsUnaccountFor.removeAll(columnsAccountFor);
        for (ColumnState columnState : columnsUnaccountFor) {
            // Check if any columns have left screen (possibly temporarily) and cull any that have been offscreen for too long
            if (columnState.currentlyVisible) {
                columnState.currentlyVisible = false;
                columnState.nonvisibleStartTime = timer.milliseconds();
            }

            columnState.deltaTimeSinceVisible = timer.milliseconds() - columnState.nonvisibleStartTime;

            if (columnState.deltaTimeSinceVisible > maxTimeNotVisible) {
                state.columnStates.remove(columnState.id);
            }
        }

        // Group into columns
        // Find biggest contour in each column
        // Figure out where the contours from the previous frame have gone using delta x center values, then update crypto state if any new contours have appeared / old ones have dropped away
        // Mark each column and center of 2 columns for debug info

        previousFrameContours = biggestContours;
        Imgproc.resize(debugImage, debugImage, initialSize);
    }

    Rectangle fromRect(Rect rect) {
        return new Rectangle(rect.x, rect.y, rect.x + rect.width, rect.y + rect.height);
    }
}
