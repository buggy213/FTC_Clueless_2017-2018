package Shared;

import Shared.vision.Contour;
import org.opencv.core.Mat;

public class Cryptobox {
    // From left to right, ascending order
    Contour[][] walls = new Contour[4][4];

    double[] averageHorizontalPositions = new double[4];

    public Cryptobox(Contour[][] walls) {
        this.walls = walls;
        for (int i = 0; i < walls.length; i++) {
            if (walls[i] != null) {
                double sum = 0;
                for (int j = 0; j < walls[i].length; j++) {
                    sum += walls[i][j].center().x;
                }
                sum /= walls[i].length;
                averageHorizontalPositions[i] = sum;
            }
            else {
                // Not detected, so use NaN
                averageHorizontalPositions[i] = Double.NaN;
            }
        }
    }

    public Contour[] getColumn(int column) {
        return walls[column];
    }

    public double[] averageHorizontalPositions() {
        return averageHorizontalPositions;
    }
}
