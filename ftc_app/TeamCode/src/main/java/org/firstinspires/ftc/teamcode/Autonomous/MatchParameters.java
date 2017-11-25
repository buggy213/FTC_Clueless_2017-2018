package org.firstinspires.ftc.teamcode.Autonomous;

import android.os.Environment;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileReader;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

/**
 * Created by hsunx on 11/24/2017.
 */

public class MatchParameters {
    Map<String, String> parameters;
    public MatchParameters() {
        parameters = new HashMap<>();
    }

    public static MatchParameters loadParameters() {
        File file = new File(Environment.getExternalStorageDirectory().getAbsolutePath(), "parameters.txt");
        //Read text from file
        MatchParameters mp = new MatchParameters();
        try {
            BufferedReader br = new BufferedReader(new FileReader(file));
            String line;

            while ((line = br.readLine()) != null) {
                String[] split = line.split(":");
                mp.parameters.put(split[0], split[1]);
            }
            br.close();
        }
        catch (IOException e) {
            //You'll need to add proper error handling here
        }

        return mp;
    }

    public String get(String key) {
        return parameters.get(key);
    }
}
