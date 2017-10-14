/*
 * Copyright (c) 2016 Arthur Pachachura, LASA Robotics, and contributors
 * MIT licensed
 */
package Shared.vision;

import org.opencv.core.Scalar;

/**
 * Implements a single color in the RGB/RGBA space
 */
public class ColorRGBA extends Color {

    /**
     * Instantiate a 3-channel (no transparency) RGB(A) color
     *
     * @param r Red channel (0-255)
     * @param g Green channel (0-255)
     * @param b Blue channel (0-255)
     */
    public ColorRGBA(int r, int g, int b) {
        super(new Scalar(r, g, b, 255));
    }

    /**
     * Instantiate a 4-channel RGBA color
     *
     * @param r Red channel (0-255)
     * @param g Green channel (0-255)
     * @param b Blue channel (0-255)
     * @param a Alpha channel (0-255), where 0 is transparent and 255 is opaque
     */
    public ColorRGBA(int r, int g, int b, int a) {
        super(new Scalar(r, g, b, a));
    }

    /**
     * Instantiate a 3- or 4-channel RGB(A) color from a Scalar value
     *
     * @param scalar Scalar value with 3-4 channels
     */
    public ColorRGBA(Scalar scalar) {
        super(scalar);
    }

    /**
     * Parse a scalar value into the colorspace
     *
     * @param s Scalar value
     * @return Colorspace scalar value
     */
    @Override
    protected Scalar parseScalar(Scalar s) {
        if (s.val.length < 3)
            throw new IllegalArgumentException("Scalar must have 3 or 4 dimensions.");

        return new Scalar(s.val[0], s.val[1], s.val[2], (s.val.length >= 4) ? (int) s.val[3] : 255);
    }

    /**
     * Get the red value
     *
     * @return Red channel (0-255)
     */
    public int red() {
        return (int) scalar.val[0];
    }

    /**
     * Get the green value
     *
     * @return Green channel (0-255)
     */
    public int green() {
        return (int) scalar.val[1];
    }

    /**
     * Get the blue value
     *
     * @return Blue channel (0-255)
     */
    public int blue() {
        return (int) scalar.val[2];
    }

    /**
     * Get the alpha value
     *
     * @return Alpha channel (0-255)
     */
    public int alpha() {
        return (int) scalar.val[3];
    }

    /**
     * Get the RGBA colorspace
     *
     * @return ColorSpace.RGBA
     */
    @Override
    public ColorSpace getColorSpace() {
        return ColorSpace.RGBA;
    }


}
