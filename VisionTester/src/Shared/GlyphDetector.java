package Shared;

import Shared.vision.Contour;
import org.opencv.core.Mat;
import org.opencv.core.Point;

import java.util.List;

enum GlyphColor {
    GREY,
    BROWN
}

public class GlyphDetector {
    public class Glyph {
        public GlyphColor color;
        public Contour outline;
        public Point center;
    }

    public static List<Glyph> detectGlyphs(Mat rgb) {
        // First find brown glyphs
    }
}
