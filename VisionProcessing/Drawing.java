/*
 * Copyright (c) 2016 Arthur Pachachura, LASA Robotics, and contributors
 * MIT licensed
 */
package org.firstinspires.ftc.teamcode.Shared.VisionProcessing;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

/**
 * Methods for drawing shapes onto images
 */
public class Drawing {
    public static void drawLine(Mat img, Point point1, Point point2, Scalar color) {
        drawLine(img, point1, point2, color, 2);
    }

    public static void drawLine(Mat img, Point point1, Point point2, Scalar color, int thickness) {
        Imgproc.line(img, point1, point2, color, thickness);
    }
}
