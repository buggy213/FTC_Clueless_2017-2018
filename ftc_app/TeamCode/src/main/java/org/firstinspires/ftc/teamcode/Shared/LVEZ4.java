package org.firstinspires.ftc.teamcode.Shared;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;

/**
 * Created by slau on 12/30/2017.
 */

public class LVEZ4 {
    private AnalogInput input;
    public LVEZ4(AnalogInput input) {
        this.input = input;
    }

    public AnalogInput getInput() {
        return this.input;
    }

    // returns distance in inches
    public double distance() {
        return (input.getVoltage() * 512);
    }
}
