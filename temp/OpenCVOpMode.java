package org.firstinspires.ftc.teamcode.Shared;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

/**
 * Created by Joshua on 9/30/2017.
 */

public class OpenCVOpMode extends OpenCVOpModeCore {
    @Override
    public Mat frame(Mat rgba, Mat gray) {
        Imgproc.cvtColor(rgba, gray, Imgproc.COLOR_RGBA2GRAY);
        return rgba;
    }
}
