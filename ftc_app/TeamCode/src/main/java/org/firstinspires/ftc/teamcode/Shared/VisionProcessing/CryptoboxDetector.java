package org.firstinspires.ftc.teamcode.Shared.VisionProcessing;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

/**
 * Created by Joshua on 9/25/2017.
 */

public class CryptoboxDetector {

    static Cryptobox cryptobox;



    public static Cryptobox getCryptobox() {
        return cryptobox;
    }

    public static void detectCryptobox(Mat rgba, Mat gray) {
        if (cryptobox == null) {
            cryptobox = new Cryptobox();
        }


    }

}
