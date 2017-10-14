package Shared.vision;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;

/**
 * Created by Joshua on 10/10/2017.
 */

public class Line {
    Point a;
    Point b;

    public Line(Point a, Point b) {
        this.a = a;
        this.b = b;
    }

    public double angle() {
        return Math.atan2((b.y - a.y), (b.x - a.x));
    }

    public boolean vertical(double maxDifferenceDegrees) {
        double deltaX = b.x - a.x;
        double deltaY = b.y - a.y;
        double angle;

        angle = Math.abs(Math.atan(deltaY / deltaX));

        double maxDifferenceRadians = (maxDifferenceDegrees / 180) * Math.PI;
        return Double.isNaN(angle) || angle > ((Math.PI / 2) - maxDifferenceRadians);
    }

    double normalize(double angle) {
        double normalizedAngle = angle;
        if (normalizedAngle < 0) {
            normalizedAngle += Math.PI * 2;
        }
        return normalizedAngle;
    }

    public void draw(Mat rgba, Scalar color, int thickness) {
        Drawing.drawLine(rgba, a, b, Color.create(color, ColorSpace.RGBA), thickness);
    }

    public double length() {
        return Math.hypot(b.y - a.y, b.x - a.x);
    }

    boolean within (double min, double max, double value) { return value > min && value < max; }
}
