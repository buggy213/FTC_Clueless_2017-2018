/*
 * Copyright (c) 2016 Arthur Pachachura, LASA Robotics, and contributors
 * MIT licensed
 */
package org.firstinspires.ftc.teamcode.Test.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

/**
 * Transform manipulation and correction
 */
public class Transform {
    /**
     * Rotate an image by an angle (counterclockwise)
     *
     * @param image Transform matrix
     * @param angle Angle to rotate by (counterclockwise) from -360 to 360
     */
    public static void rotate(Mat image, double angle) {
        //Calculate size of new matrix
        double radians = Math.toRadians(angle);
        double sin = Math.abs(Math.sin(radians));
        double cos = Math.abs(Math.cos(radians));

        int newWidth = (int) (image.width() * cos + image.height() * sin);
        int newHeight = (int) (image.width() * sin + image.height() * cos);

        // rotating image
        Point center = new Point(newWidth / 2, newHeight / 2);
        Mat rotMatrix = Imgproc.getRotationMatrix2D(center, angle, 1.0); //1.0 means 100 % scale

        Size size = new Size(newWidth, newHeight);
        Imgproc.warpAffine(image, image, rotMatrix, image.size());
    }

    public static void flip(Mat img, FlipType flipType) {
        Core.flip(img, img, flipType.val);
    }

    private static double makeScale(Mat img, Size approxSize, boolean maximize, boolean integerScale) {
        Size imageSize = img.size();
        double ratioWidth = approxSize.width / imageSize.width;
        double ratioHeight = approxSize.height / imageSize.height;
        double ratio = maximize ? Math.max(ratioWidth, ratioHeight) : Math.min(ratioWidth, ratioHeight);
        if (MathUtil.equal(ratio, 1))
            return 1;
        if (integerScale) {
            //The scale factor is always greater than 1
            double scale = (ratio < 1) ? 1 / ratio : ratio;
            //If you are actually increasing the size of the object, use ceiling()
            //Otherwise, use floor()
            scale = maximize ^ (ratio < 1) ? Math.ceil(scale) : Math.floor(scale);
            //Get the actual ratio again
            return (ratio < 1) ? 1 / scale : scale;
        } else {
            return ratio;
        }
    }

    /**
     * Type of reflection
     * Creates a reflection matrix based on the type of value
     */
    public enum FlipType {
        //Reflect across Y axis
        FLIP_ACROSS_Y(0),
        //Reflect across X axis
        FLIP_ACROSS_X(1),
        //Reflect across both axis simultaneously
        FLIP_BOTH(-1);

        final int val;

        FlipType(int a) {
            this.val = a;
        }
    }
}
