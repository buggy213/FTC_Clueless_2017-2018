package org.firstinspires.ftc.teamcode.Shared;

import android.content.Context;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;

/**
 * Created by hsunx on 11/22/2017.
 */

public class Debug {
    static StringBuilder sb;
    public static void init() {
        sb = new StringBuilder();
    }

    public static void addLine(String s) {
        sb.append(s + "\n");
    }

    public static void close() {
        File file = new File(AppUtil.getInstance().getActivity().getFilesDir(), "log.txt");
        FileOutputStream out = null;

        try {
            out = AppUtil.getInstance().getActivity().openFileOutput("log.txt", Context.MODE_PRIVATE);
            out.write(sb.toString().getBytes());
            out.close();
        }
        catch (Exception e) {
            return;
        }


    }
}
