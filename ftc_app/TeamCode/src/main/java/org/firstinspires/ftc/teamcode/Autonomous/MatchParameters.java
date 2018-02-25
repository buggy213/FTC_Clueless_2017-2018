package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.InputStreamReader;
import java.util.HashMap;
import java.util.Map;

/**
 * Created by hsunx on 11/24/2017.
 */

// OpMode interface to the MatchParameters
public class MatchParameters {
    Map<String, String> parameters;
    public MatchParameters() {
        parameters = new HashMap<>();
    }
    public static MatchParameters current;
    public static void initCurrentParams() {
        current = new MatchParameters();
    }
    public static MatchParameters loadParameters(String data) {
        //Read text from file, parse and store in a Map (Dictionary)
        MatchParameters mp = new MatchParameters();
        String[] lines = data.split("\\n");
        for (int i = 0; i < lines.length; i++) {
            String[] results = lines[i].split(":");
            mp.put(results[0], results[1]);
        }
        return mp;
    }

    public static MatchParameters loadParametersFromDefaultLocation() {
        File file = new File(AppUtil.getDefContext().getFilesDir().getAbsolutePath() + "/Settings/matchparams.txt");
        StringBuilder sb;
        try {
            FileInputStream fis = new FileInputStream(file);
            InputStreamReader isr = new InputStreamReader(fis);
            BufferedReader bufferedReader = new BufferedReader(isr);
            sb = new StringBuilder();
            String line;
            while ((line = bufferedReader.readLine()) != null) {
                sb.append(line);
            }
        }
        catch (Exception e) {
            return null;
        }

        return loadParameters(sb.toString());
    }

    public void put(String key, String value) {
        parameters.put(key, value);
    }

    public String get(String key) {
        return parameters.get(key);
    }

    public int getInt(String key) {
        return Integer.valueOf(parameters.get(key.trim()));
    }

    public boolean getBool(String key) {return Boolean.valueOf(parameters.get(key.trim().toLowerCase())); }
}
