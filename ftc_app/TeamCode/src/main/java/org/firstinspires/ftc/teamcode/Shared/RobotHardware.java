package org.firstinspires.ftc.teamcode.Shared;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;

import java.lang.reflect.Array;
import java.lang.reflect.Field;
import java.util.ArrayList;

/**
 * Created by Joshua on 9/9/2017.
 */

public class RobotHardware {

    private static RobotHardware hw;

    private HardwareMap hwMap;

    public DcMotor forwardLeft;
    public DcMotor forwardRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    public GyroSensor imu;


    public static RobotHardware GetSingleton(HardwareMap map) {
        if (hw == null) {
            hw = new RobotHardware(map);
        }
        return hw;
    }

    public static RobotHardware GetSingleton() {
        if (hw == null) {
            throw new RuntimeException();
        }

        return hw;
    }

    public RobotHardware(HardwareMap map) {
        this.hwMap = map;

        //TODO revisit reflection mapping
        /*
        Field[] allFields = this.getClass().getDeclaredFields();
        ArrayList<Field> deviceFields = new ArrayList<Field>();

        for (Field field : allFields) {
            Log.d("Map", field.getName() + "|" + field.getType().toString());
            try {
                if (field.get(this)) {
                    deviceFields.add(field);
                }
            } catch (Exception e) {
                Log.e("HardwareMap", e.toString());
            }
        }
        for (Field f : deviceFields) {
            Log.d("Map", f.getName() + f.getType().toString());
            try {
                f.set(this, map.get(f.getName()));
            } catch (Exception e) {
                Log.e("HardwareMap", e.toString());
            }
        }
*/
        forwardLeft = map.dcMotor.get("forwardLeft");
        forwardRight = map.dcMotor.get("forwardRight");
        backLeft = map.dcMotor.get("backLeft");
        backRight = map.dcMotor.get("backRight");

        forwardLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);


    }

}
