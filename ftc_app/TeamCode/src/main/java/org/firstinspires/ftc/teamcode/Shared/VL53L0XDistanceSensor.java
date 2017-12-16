package org.firstinspires.ftc.teamcode.Shared;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;

/**
 * Created by hsunx on 12/15/2017.
 */
@I2cSensor(name = "VL53L0X Time-of-Flight Sensor", description = "Time-of-flight (distance) sensor", xmlTag = "VL53L0X")
public class VL53L0XDistanceSensor extends I2cDeviceSynchDevice<I2cDeviceSynch> {
    int bswap(int val) {
        // Cull top 32 bytes
        val = val & 0xffff;
        int upper = val & 0x0000ff00;
        int lower = val & 0x000000ff;
        upper = upper >> 8;
        lower = lower << 8;
        return (upper | lower);
    }
    int VL53L0X_decode_vcsel_period(int vcsel_period_reg) {
        int vcsel_period_pclks = (vcsel_period_reg + 1) << 1;
        return vcsel_period_pclks;
    }

    enum Register {
        VL53L0X_REG_IDENTIFICATION_MODEL_ID(0x00c0),
        VL53L0X_REG_IDENTIFICATION_REVISION_ID(0x00c2),
        VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD(0x0050),
        VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD(0x0070),
        VL53L0X_REG_SYSRANGE_START(0x000),

        VL53L0X_REG_RESULT_INTERRUPT_STATUS(0x0013),
        VL53L0X_REG_RESULT_RANGE_STATUS(0x0014);

        public int bVal;
        Register(int bVal) {
            this.bVal = bVal;
        }
    }

    I2cAddr ADDRESS_I2C_DEFAULT = new I2cAddr(0x29);

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
        return "VL53L0X Time-of-Flight Sensor";
    }

    public int getDistance() {
        return 0;
    }

    public VL53L0XDistanceSensor(I2cDeviceSynch deviceClient) {
        super(deviceClient, true);

        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
        this.deviceClient.read()


    }
}
