package org.firstinspires.ftc.teamcode.Shared;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;

import java.io.IOException;

/*
 * I2C Time of Flight distance sensor, 30 to 1000 mm.
 * https://www.adafruit.com/products/3317
 *
 * Basic implementation
 */
@I2cSensor(name = "VL53L0X Time-of-Flight Sensor", description = "Time-of-flight (distance) sensor", xmlTag = "VL53L0X")
public class VL53L0X extends I2cDeviceSynchDevice<I2cDeviceSynch>{
    public final static int VL53L0X_I2CADDR = 0x29;

    private final static int VL53L0X_REG_IDENTIFICATION_MODEL_ID = 0x00c0;
    private final static int VL53L0X_REG_IDENTIFICATION_REVISION_ID = 0x00c2;
    private final static int VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD = 0x0050;
    private final static int VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD = 0x0070;
    private final static int VL53L0X_REG_SYSRANGE_START = 0x000;

    private final static int VL53L0X_REG_RESULT_INTERRUPT_STATUS = 0x0013;
    private final static int VL53L0X_REG_RESULT_RANGE_STATUS = 0x0014;

    public final static int VL53L0X_GOOD_ACCURACY_MODE      = 0;   // Good Accuracy mode
    public final static int VL53L0X_BETTER_ACCURACY_MODE    = 1;   // Better Accuracy mode
    public final static int VL53L0X_BEST_ACCURACY_MODE      = 2;   // Best Accuracy mode
    public final static int VL53L0X_LONG_RANGE_MODE         = 3;   // Longe Range mode
    public final static int VL53L0X_HIGH_SPEED_MODE         = 4;   // High Speed mode

    private static boolean verbose = true;


    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Adafruit;
    }

    @Override
    protected boolean doInitialize() {
        return true;
    }

    @Override
    public String getDeviceName() {
        return "VL53L0X";
    }

    public VL53L0X(I2cDeviceSynch deviceClient) {
        super(deviceClient, true);

        this.deviceClient.setI2cAddress(new I2cAddr(VL53L0X_I2CADDR));
        setOptimalReadWindow();
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    protected void setOptimalReadWindow() {
        this.deviceClient.setReadWindow(new I2cDeviceSynch.ReadWindow(0x14, 12, I2cDeviceSynch.ReadMode.REPEAT));
    }

    public int getRevision() {
        int revision = 0;
        try {
            revision = this.deviceClient.read8(VL53L0X_REG_IDENTIFICATION_REVISION_ID);
        } catch (Exception e) {
            e.printStackTrace();
        }
        return revision;
    }

    public int getDeviceID() {
        int revision = 0;
        try {
            revision = this.deviceClient.read8(VL53L0X_REG_IDENTIFICATION_MODEL_ID);
        } catch (Exception e) {
            e.printStackTrace();
        }
        return revision;
    }

    public void startRanging(int mode) throws Exception {
        if (mode >= VL53L0X_GOOD_ACCURACY_MODE && mode <= VL53L0X_HIGH_SPEED_MODE) {
            this.deviceClient.write8(VL53L0X_REG_SYSRANGE_START, 0x01);
            // Waiting for the device to be ready
            final int NB_TRY = 100; // 1 sec max
            boolean ok = false;
            int nb = 0;
            int value = 0;
            while (!ok && nb++ < NB_TRY) {
                try { Thread.sleep(10); } catch (InterruptedException ie) {} // 10ms
                value = this.deviceClient.read8(VL53L0X_REG_RESULT_RANGE_STATUS);
                if ((value & 0x01) == 0x01) {
                    ok = true;
                    if (verbose) {
                        System.out.println("Device ready");
                    }
                }
            }
            if ((value & 0x01) != 0x01) {
                // Not ready
                if (verbose) {
                    System.out.println("Device NOT ready");
                }
                throw new RuntimeException("Device not ready");
            }
        } // TODO else (IllegalPrmException)
    }

    public void stopRanging() {
        // TODO Implement
    }

    public VL53L0XData getVL53L0XData() throws Exception {
        byte[] data = readBlockData(0x14, 12);

        if (verbose) {
            StringBuffer sb = new StringBuffer(); // No way to stream a byte[] ... :(
            for (byte b : data) {
                sb.append(String.format("%02X ", b));
            }
            System.out.println(sb.toString().trim());
        }

        int ambientCount = ((data[6] & 0xFF) << 8) | (data[7] & 0xFF);
        int signalCount = ((data[8] & 0xFF) << 8) | (data[9] & 0xFF);
        int distance = ((data[10] & 0xFF) << 8) | (data[11] & 0xFF);
        int deviceRangeStatusInternal = ((data[0] & 0x78) >> 3);

        return new VL53L0XData(ambientCount, signalCount, deviceRangeStatusInternal, distance);
    }

    public int getDistance() throws Exception {
        return this.getVL53L0XData().distance;
    }

    private static class VL53L0XData {
        int ambientCount;
        int signalCount;
        int distance;
        int deviceRangeStatusInternal;

        public VL53L0XData(int ambientCount, int signalCount, int deviceRangeStatusInternal, int distance) {
            this.ambientCount = ambientCount;
            this.signalCount = signalCount;
            this.deviceRangeStatusInternal = deviceRangeStatusInternal;
            this.distance = distance;
        }
    }

    private byte[] readBlockData(int register, int nb) throws IOException {
        return this.deviceClient.read(register, nb);
    }
}