package org.firstinspires.ftc.teamcode.Shared;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.RobotLog;

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

    public DcMotor linearSlidePivotMotor;
    public DcMotor linearSlideDriveMotor;

    public DcMotor clawMotor;

    public Servo phoneServo1;
    public Servo phoneServo2;

    public Servo jewelArm1;
    public Servo jewelArm2;

    public DcMotor beltMotor;

    public ColorSensor left_color;
    public ColorSensor right_color;

    public DistanceSensor sensorDistance;
    public BNO055IMU imu;

    public void ReinitializeIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

    }

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

        Field[] allFields = this.getClass().getDeclaredFields();
        for (Field field : allFields) {
            RobotLog.i(field.getName() + ":" + field.getType());
            if (HardwareDevice.class.isAssignableFrom(field.getType())) {

                // Hardware device, try to assign
                try {
                    field.set(this, hwMap.get(field.getType(), field.getName()));
                }
                catch (IllegalAccessException e) {
                    RobotLog.e("Error during reflection mapping");
                }
            }
        }
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



        /*forwardLeft = map.dcMotor.get("forwardLeft");
        forwardRight = map.dcMotor.get("forwardRight");
        backLeft = map.dcMotor.get("backLeft");
        backRight = map.dcMotor.get("backRight");

        linearSlidePivotMotor = map.dcMotor.get("linearSlidePivotMotor");
        linearSlideDriveMotor = map.dcMotor.get("linearSlideDriveMotor");

        clawServo1 = map.servo.get("clawServo1");
        clawServo2 = map.servo.get("clawServo2");

        relicClawServo = map.crservo.get("relicClawServo");

        beltMotor = map.dcMotor.get("beltMotor");

        phoneServo1 = map.servo.get("phoneMount1");
        phoneServo2 = map.servo.get("phoneMount2");

        sensorDistance = map.get(DistanceSensor.class, "distanceColorSensor");
*/
        forwardRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        forwardLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        forwardRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

}
