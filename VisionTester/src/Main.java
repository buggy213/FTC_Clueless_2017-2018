import Shared.CryptoboxDetector;
import Shared.GlyphDetector;
import Shared.JewelDetector;
import Shared.TeamColor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

public class Main {
    public static void main(String[] args) {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
        Mat rgb = Imgcodecs.imread("imgs/test.jpg");
        Imgcodecs.imread("imgs/test.jpg", Imgcodecs.IMREAD_COLOR);
        Mat grey = new Mat();
        Imgproc.cvtColor(rgb, rgb, Imgproc.COLOR_BGR2RGB);
        Imgproc.cvtColor(rgb, grey, Imgproc.COLOR_RGB2GRAY);
        JewelDetector jd = new JewelDetector();
        jd.processFrame(rgb, grey);
    }
}
