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

    public boolean parallel(Line other, double maxDifferenceDegrees) {
        double theta1 = normalize(angle());
        double theta2 = normalize(other.angle());

        double maxDifferenceRadians = (maxDifferenceDegrees / 180) * Math.PI;
        double difference = (((theta1 - theta2) + 3 * Math.PI) % (2 * Math.PI)) - Math.PI;

        return (difference < maxDifferenceRadians && difference > -maxDifferenceRadians);
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
}
