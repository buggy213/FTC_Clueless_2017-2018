package org.firstinspires.ftc.teamcode.Shared.VisionProcessing;

import org.opencv.core.Point;

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
        double rotatedAngle = Math.atan2((b.x - a.x), (b.y - a.y));
        double maxDifferenceRadians = (maxDifferenceDegrees / 180) * Math.PI;
        return within(-maxDifferenceRadians, maxDifferenceRadians, rotatedAngle);
    }

    double normalize(double angle) {
        double normalizedAngle = angle;
        if (normalizedAngle < 0) {
            normalizedAngle += Math.PI * 2;
        }
        return normalizedAngle;
    }

    public double length() {
        return Math.hypot(b.y - a.y, b.x - a.x);
    }

    boolean within (double min, double max, double value) { return value > min && value < max; }
}
