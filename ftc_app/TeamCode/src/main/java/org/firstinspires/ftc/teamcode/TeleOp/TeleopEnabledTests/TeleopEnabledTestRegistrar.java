package org.firstinspires.ftc.teamcode.TeleOp.TeleopEnabledTests;


import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by Joshua on 2/8/2018.
 */

public class TeleopEnabledTestRegistrar {
    public static List<Class<? extends TeleopEnabledTest>> teleopEnabledTestClasses = new ArrayList<>();

    // Add/remove tests here
    public static void LoadTests() {
        teleopEnabledTestClasses.add(TurnTest.class);
        teleopEnabledTestClasses.add(ServosAndMotorsTest.class);
        teleopEnabledTestClasses.add(AutoGrabTest.class);
    }

}
