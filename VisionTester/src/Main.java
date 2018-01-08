import Shared.CryptoboxDetector;
import Shared.GlyphDetector;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

public class Main {
    public static void main(String[] args) {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
        Mat rgb = Imgcodecs.imread("imgs/test.jpg");
        Mat grey = new Mat();
        Imgproc.cvtColor(rgb, grey, Imgproc.COLOR_RGBA2GRAY);
        GlyphDetector gd = new GlyphDetector();
        gd.processFrame(rgb, grey);
    }
}
