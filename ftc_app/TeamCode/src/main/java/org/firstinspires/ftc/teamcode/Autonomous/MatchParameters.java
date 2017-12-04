package org.firstinspires.ftc.teamcode.Autonomous;

import android.os.Environment;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileReader;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;
import java.util.PriorityQueue;

/**
 * Created by hsunx on 11/24/2017.
 */

// OpMode interface to the MatchParameters
public class MatchParameters {
    Map<String, String> parameters;
    public MatchParameters() {
        parameters = new HashMap<>();
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

    public void put(String key, String value) {
        parameters.put(key, value);
    }

    public String get(String key) {
        return parameters.get(key);
    }

    public int getInt(String key) {
        return Integer.valueOf(parameters.get(key));
    }
}
