package org.firstinspires.ftc.teamcode.Shared.Sensors.VL53L0X;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.TypeConversion;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

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
    int measurement_timing_budget_us = 0;
    ElapsedTime elapsedTime;
    int SYSRANGE_START                              = 0x00;

    int SYSTEM_THRESH_HIGH                          = 0x0C;
    int SYSTEM_THRESH_LOW                           = 0x0E;

    int SYSTEM_SEQUENCE_CONFIG                      = 0x01;
    int SYSTEM_RANGE_CONFIG                         = 0x09;
    int SYSTEM_INTERMEASUREMENT_PERIOD              = 0x04;

    int SYSTEM_INTERRUPT_CONFIG_GPIO                = 0x0A;

    int GPIO_HV_MUX_ACTIVE_HIGH                     = 0x84;

    int SYSTEM_INTERRUPT_CLEAR                      = 0x0B;

    int RESULT_INTERRUPT_STATUS                     = 0x13;
    int RESULT_RANGE_STATUS                         = 0x14;

    int RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN       = 0xBC;
    int RESULT_CORE_RANGING_TOTAL_EVENTS_RTN        = 0xC0;
    int RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF       = 0xD0;
    int RESULT_CORE_RANGING_TOTAL_EVENTS_REF        = 0xD4;
    int RESULT_PEAK_SIGNAL_RATE_REF                 = 0xB6;

    int ALGO_PART_TO_PART_RANGE_OFFSET_MM           = 0x28;

    int I2C_SLAVE_DEVICE_ADDRESS                    = 0x8A;

    int MSRC_CONFIG_CONTROL                         = 0x60;

    int PRE_RANGE_CONFIG_MIN_SNR                    = 0x27;
    int PRE_RANGE_CONFIG_VALID_PHASE_LOW            = 0x56;
    int PRE_RANGE_CONFIG_VALID_PHASE_HIGH           = 0x57;
    int PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT          = 0x64;

    int FINAL_RANGE_CONFIG_MIN_SNR                  = 0x67;
    int FINAL_RANGE_CONFIG_VALID_PHASE_LOW          = 0x47;
    int FINAL_RANGE_CONFIG_VALID_PHASE_HIGH         = 0x48;
    int FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44;

    int PRE_RANGE_CONFIG_SIGMA_THRESH_HI            = 0x61;
    int PRE_RANGE_CONFIG_SIGMA_THRESH_LO            = 0x62;

    int PRE_RANGE_CONFIG_VCSEL_PERIOD               = 0x50;
    int PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI          = 0x51;
    int PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO          = 0x52;

    int SYSTEM_HISTOGRAM_BIN                        = 0x81;
    int HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT       = 0x33;
    int HISTOGRAM_CONFIG_READOUT_CTRL               = 0x55;

    int FINAL_RANGE_CONFIG_VCSEL_PERIOD             = 0x70;
    int FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI        = 0x71;
    int FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO        = 0x72;
    int CROSSTALK_COMPENSATION_PEAK_RATE_MCPS       = 0x20;

    int MSRC_CONFIG_TIMEOUT_MACROP                  = 0x46;

    int SOFT_RESET_GO2_SOFT_RESET_N                 = 0xBF;
    int IDENTIFICATION_MODEL_ID                     = 0xC0;
    int IDENTIFICATION_REVISION_ID                  = 0xC2;

    int OSC_CALIBRATE_VAL                           = 0xF8;

    int GLOBAL_CONFIG_VCSEL_WIDTH                   = 0x32;
    int GLOBAL_CONFIG_SPAD_ENABLES_REF_0            = 0xB0;
    int GLOBAL_CONFIG_SPAD_ENABLES_REF_1            = 0xB1;
    int GLOBAL_CONFIG_SPAD_ENABLES_REF_2            = 0xB2;
    int GLOBAL_CONFIG_SPAD_ENABLES_REF_3            = 0xB3;
    int GLOBAL_CONFIG_SPAD_ENABLES_REF_4            = 0xB4;
    int GLOBAL_CONFIG_SPAD_ENABLES_REF_5            = 0xB5;

    int GLOBAL_CONFIG_REF_EN_START_SELECT           = 0xB6;
    int DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD         = 0x4E;
    int DYNAMIC_SPAD_REF_EN_START_OFFSET            = 0x4F;
    int POWER_MANAGEMENT_GO1_POWER_FORCE            = 0x80;

    int VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV           = 0x89;

    int ALGO_PHASECAL_LIM                           = 0x30;
    int ALGO_PHASECAL_CONFIG_TIMEOUT                = 0x30;

    int stop_variable;

    I2cAddr ADDRESS_I2C_DEFAULT = new I2cAddr(0x29);
    int timeoutStartMs;
    int timeoutMs = 0;

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

    int lastKnownRange = 0;

    public int getLastKnownRange() {
        return lastKnownRange;
    }

    boolean checkTimeoutExpired() {
        return timeoutMs > 0 && (elapsedTime.milliseconds() - timeoutStartMs) > timeoutMs;
    }

    public int getDistance() {
        return 0;
    }

    public VL53L0XDistanceSensor(I2cDeviceSynch deviceClient) {
        super(deviceClient, true);

        elapsedTime = new ElapsedTime();
        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    void writeReg(int register, int value) {
        this.deviceClient.write8(register, value);
    }

    // Write a 16-bit register
    void writeReg16Bit(int reg, int value)
    {
        this.deviceClient.write8(reg, (value >> 8) & 0xFF);
        this.deviceClient.write8(reg + 0xFF, (value) & 0xFF);
    }

    // Set the return signal rate limit check value in units of MCPS (mega counts
    // per second). "This represents the amplitude of the signal reflected from the
    // target and detected by the device"; setting this limit presumably determines
    // the minimum measurement necessary for the sensor to report a valid reading.
    // Setting a lower limit increases the potential range of the sensor but also
    // seems to increase the likelihood of getting an inaccurate reading because of
    // unwanted reflections from objects other than the intended target.
    // Defaults to 0.25 MCPS as initialized by the ST API and this library.
    boolean setSignalRateLimit(float limit_Mcps)
    {
        if (limit_Mcps < 0 || limit_Mcps > 511.99) { return false; }

        // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
        writeReg16Bit(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, Float.floatToIntBits(limit_Mcps * (1 << 7)));
        return true;
    }
    // Get reference SPAD (single photon avalanche diode) count and type
    // based on VL53L0X_get_info_from_device(),
    // but only gets reference SPAD count and type
    boolean getSpadInfo(AtomicInteger count, AtomicBoolean type_is_aperture)
    {
        int tmp;

        writeReg(0x80, 0x01);
        writeReg(0xFF, 0x01);
        writeReg(0x00, 0x00);

        writeReg(0xFF, 0x06);
        writeReg(0x83, readReg(0x83) | 0x04);
        writeReg(0xFF, 0x07);
        writeReg(0x81, 0x01);

        writeReg(0x80, 0x01);

        writeReg(0x94, 0x6b);
        writeReg(0x83, 0x00);
        startTimeout();
        while (readReg(0x83) == 0x00)
        {
            if (checkTimeoutExpired()) { return false; }
        }
        writeReg(0x83, 0x01);
        tmp = readReg(0x92);

        count.set(tmp & 0x7f);
        type_is_aperture.set(((tmp >> 7) & 0x01) == 0x01);

        writeReg(0x81, 0x00);
        writeReg(0xFF, 0x06);
        writeReg(0x83, readReg(0x83)  & ~0x04);
        writeReg(0xFF, 0x01);
        writeReg(0x00, 0x01);

        writeReg(0xFF, 0x00);
        writeReg(0x80, 0x00);

        return true;
    }

    // Initialize sensor using sequence based on VL53L0X_DataInit(),
    // VL53L0X_StaticInit(), and VL53L0X_PerformRefCalibration().
    // This function does not perform reference SPAD calibration
    // (VL53L0X_PerformRefSpadManagement()), since the API user manual says that it
    // is performed by ST on the bare modules; it seems like that should work well
    // enough unless a cover glass is added.
    // If io_2v8 (optional) is true or not given, the sensor is configured for 2V8
    // mode.

    boolean init(boolean io_2v8)
    {
        // VL53L0X_DataInit() begin

        // sensor uses 1V8 mode for I/O by default; switch to 2V8 mode if necessary
        if (io_2v8)
        {
            writeReg(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,
                    readReg(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01); // set bit 0
        }

        // "Set I2C standard mode"
        writeReg(0x88, 0x00);

        writeReg(0x80, 0x01);
        writeReg(0xFF, 0x01);
        writeReg(0x00, 0x00);
        stop_variable = readReg(0x91);
        writeReg(0x00, 0x01);
        writeReg(0xFF, 0x00);
        writeReg(0x80, 0x00);

        // disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
        writeReg(MSRC_CONFIG_CONTROL, readReg(MSRC_CONFIG_CONTROL) | 0x12);

        // set final range signal rate limit to 0.25 MCPS (million counts per second)
        setSignalRateLimit(0.25f);

        writeReg(SYSTEM_SEQUENCE_CONFIG, 0xFF);

        // VL53L0X_DataInit() end

        // VL53L0X_StaticInit() begin

        AtomicInteger spad_count = new AtomicInteger(0);
        AtomicBoolean spad_type_is_aperture = new AtomicBoolean(false);
        if (!getSpadInfo(spad_count, spad_type_is_aperture)) { return false; }

        // The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
        // the API, but the same data seems to be more easily readable from
        // GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
        byte[] ref_spad_map;
        ref_spad_map = this.deviceClient.read(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, 6);

        // -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)

        writeReg(0xFF, 0x01);
        writeReg(DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
        writeReg(DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
        writeReg(0xFF, 0x00);
        writeReg(GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

        int first_spad_to_enable = spad_type_is_aperture.get() ? 12 : 0; // 12 is the first aperture spad
        int spads_enabled = 0;

        for (int i = 0; i < 48; i++)
        {
            if (i < first_spad_to_enable || spads_enabled == spad_count.get())
            {
                // This bit is lower than the first one that should be enabled, or
                // (reference_spad_count) bits have already been enabled, so zero this bit
                ref_spad_map[i / 8] &= ~(1 << (i % 8));
            }
            else if (((ref_spad_map[i / 8] >> (i % 8)) & 0x1) == 0x1)
            {
                spads_enabled++;
            }
        }

        this.deviceClient.write(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map);

        // -- VL53L0X_set_reference_spads() end

        // -- VL53L0X_load_tuning_settings() begin
        // DefaultTuningSettings from vl53l0x_tuning.h

        writeReg(0xFF, 0x01);
        writeReg(0x00, 0x00);

        writeReg(0xFF, 0x00);
        writeReg(0x09, 0x00);
        writeReg(0x10, 0x00);
        writeReg(0x11, 0x00);

        writeReg(0x24, 0x01);
        writeReg(0x25, 0xFF);
        writeReg(0x75, 0x00);

        writeReg(0xFF, 0x01);
        writeReg(0x4E, 0x2C);
        writeReg(0x48, 0x00);
        writeReg(0x30, 0x20);

        writeReg(0xFF, 0x00);
        writeReg(0x30, 0x09);
        writeReg(0x54, 0x00);
        writeReg(0x31, 0x04);
        writeReg(0x32, 0x03);
        writeReg(0x40, 0x83);
        writeReg(0x46, 0x25);
        writeReg(0x60, 0x00);
        writeReg(0x27, 0x00);
        writeReg(0x50, 0x06);
        writeReg(0x51, 0x00);
        writeReg(0x52, 0x96);
        writeReg(0x56, 0x08);
        writeReg(0x57, 0x30);
        writeReg(0x61, 0x00);
        writeReg(0x62, 0x00);
        writeReg(0x64, 0x00);
        writeReg(0x65, 0x00);
        writeReg(0x66, 0xA0);

        writeReg(0xFF, 0x01);
        writeReg(0x22, 0x32);
        writeReg(0x47, 0x14);
        writeReg(0x49, 0xFF);
        writeReg(0x4A, 0x00);

        writeReg(0xFF, 0x00);
        writeReg(0x7A, 0x0A);
        writeReg(0x7B, 0x00);
        writeReg(0x78, 0x21);

        writeReg(0xFF, 0x01);
        writeReg(0x23, 0x34);
        writeReg(0x42, 0x00);
        writeReg(0x44, 0xFF);
        writeReg(0x45, 0x26);
        writeReg(0x46, 0x05);
        writeReg(0x40, 0x40);
        writeReg(0x0E, 0x06);
        writeReg(0x20, 0x1A);
        writeReg(0x43, 0x40);

        writeReg(0xFF, 0x00);
        writeReg(0x34, 0x03);
        writeReg(0x35, 0x44);

        writeReg(0xFF, 0x01);
        writeReg(0x31, 0x04);
        writeReg(0x4B, 0x09);
        writeReg(0x4C, 0x05);
        writeReg(0x4D, 0x04);

        writeReg(0xFF, 0x00);
        writeReg(0x44, 0x00);
        writeReg(0x45, 0x20);
        writeReg(0x47, 0x08);
        writeReg(0x48, 0x28);
        writeReg(0x67, 0x00);
        writeReg(0x70, 0x04);
        writeReg(0x71, 0x01);
        writeReg(0x72, 0xFE);
        writeReg(0x76, 0x00);
        writeReg(0x77, 0x00);

        writeReg(0xFF, 0x01);
        writeReg(0x0D, 0x01);

        writeReg(0xFF, 0x00);
        writeReg(0x80, 0x01);
        writeReg(0x01, 0xF8);

        writeReg(0xFF, 0x01);
        writeReg(0x8E, 0x01);
        writeReg(0x00, 0x01);
        writeReg(0xFF, 0x00);
        writeReg(0x80, 0x00);

        // -- VL53L0X_load_tuning_settings() end

        // "Set interrupt config to new sample ready"
        // -- VL53L0X_SetGpioConfig() begin

        writeReg(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
        writeReg(GPIO_HV_MUX_ACTIVE_HIGH, readReg(GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10); // active low
        writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);

        // -- VL53L0X_SetGpioConfig() end

        measurement_timing_budget_us = getMeasurementTimingBudget();

        // "Disable MSRC and TCC by default"
        // MSRC = Minimum Signal Rate Check
        // TCC = Target CentreCheck
        // -- VL53L0X_SetSequenceStepEnable() begin

        writeReg(SYSTEM_SEQUENCE_CONFIG, 0xE8);

        // -- VL53L0X_SetSequenceStepEnable() end

        // "Recalculate timing budget"
        setMeasurementTimingBudget(measurement_timing_budget_us);

        // VL53L0X_StaticInit() end

        // VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())

        // -- VL53L0X_perform_vhv_calibration() begin

        writeReg(SYSTEM_SEQUENCE_CONFIG, 0x01);
        if (!performSingleRefCalibration(0x40)) { return false; }

        // -- VL53L0X_perform_vhv_calibration() end

        // -- VL53L0X_perform_phase_calibration() begin

        writeReg(SYSTEM_SEQUENCE_CONFIG, 0x02);
        if (!performSingleRefCalibration(0x00)) { return false; }

        // -- VL53L0X_perform_phase_calibration() end

        // "restore the previous Sequence Config"
        writeReg(SYSTEM_SEQUENCE_CONFIG, 0xE8);

        // VL53L0X_PerformRefCalibration() end

        return true;
    }

    // Get sequence step enables
    // based on VL53L0X_GetSequenceStepEnables()
    void getSequenceStepEnables(AtomicReference<SequenceStepEnables> enables)
    {
        int sequence_config = readReg(SYSTEM_SEQUENCE_CONFIG);
        SequenceStepEnables copy = enables.get();
        copy.tcc          = ((sequence_config >> 4) & 0x1) == 0x1;
        copy.dss          = ((sequence_config >> 3) & 0x1) == 0x1;
        copy.msrc         = ((sequence_config >> 2) & 0x1) == 0x1;
        copy.pre_range    = ((sequence_config >> 6) & 0x1) == 0x1;
        copy.final_range  = ((sequence_config >> 7) & 0x1) == 0x1;
        enables.set(copy);
    }
    // Set the measurement timing budget in microseconds, which is the time allowed
    // for one measurement; the ST API and this library take care of splitting the
    // timing budget among the sub-steps in the ranging sequence. A longer timing
    // budget allows for more accurate measurements. Increasing the budget by a
    // factor of N decreases the range measurement standard deviation by a factor of
    // sqrt(N). Defaults to about 33 milliseconds; the minimum is 20 ms.
    // based on VL53L0X_set_measurement_timing_budget_micro_seconds()
    boolean setMeasurementTimingBudget(int budget_us)
    {
        AtomicReference<SequenceStepEnables> enables = new AtomicReference<>(new SequenceStepEnables());
        AtomicReference<SequenceStepTimeouts> timeouts = new AtomicReference<>(new SequenceStepTimeouts());

        int StartOverhead      = 1320; // note that this is different than the value in get_
        int EndOverhead        = 960;
        int MsrcOverhead       = 660;
        int TccOverhead        = 590;
        int DssOverhead        = 690;
        int PreRangeOverhead   = 660;
        int FinalRangeOverhead = 550;

        int MinTimingBudget = 20000;

        if (budget_us < MinTimingBudget) { return false; }

        int used_budget_us = StartOverhead + EndOverhead;

        getSequenceStepEnables(enables);
        getSequenceStepTimeouts(enables, timeouts);

        SequenceStepEnables e = enables.get();
        SequenceStepTimeouts t = timeouts.get();
        if (e.tcc)
        {
            used_budget_us += (t.msrc_dss_tcc_us + TccOverhead);
        }

        if (e.dss)
        {
            used_budget_us += 2 * (t.msrc_dss_tcc_us + DssOverhead);
        }
        else if (e.msrc)
        {
            used_budget_us += (t.msrc_dss_tcc_us + MsrcOverhead);
        }

        if (e.pre_range)
        {
            used_budget_us += (t.pre_range_us + PreRangeOverhead);
        }

        if (e.final_range)
        {
            used_budget_us += FinalRangeOverhead;

            // "Note that the final range timeout is determined by the timing
            // budget and the sum of all other timeouts within the sequence.
            // If there is no room for the final range timeout, then an error
            // will be set. Otherwise the remaining time will be applied to
            // the final range."

            if (used_budget_us > budget_us)
            {
                // "Requested timeout too big."
                return false;
            }

            int final_range_timeout_us = budget_us - used_budget_us;

            // set_sequence_step_timeout() begin
            // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

            // "For the final range timeout, the pre-range timeout
            //  must be added. To do this both final and pre-range
            //  timeouts must be expressed in macro periods MClks
            //  because they have different vcsel periods."

            int final_range_timeout_mclks =
                    timeoutMicrosecondsToMclks(final_range_timeout_us,
                            t.final_range_vcsel_period_pclks);

            if (e.pre_range)
            {
                final_range_timeout_mclks += t.pre_range_mclks;
            }

            writeReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                    encodeTimeout(final_range_timeout_mclks));

            // set_sequence_step_timeout() end

            measurement_timing_budget_us = budget_us; // store for internal reuse
        }
        return true;
    }

    // Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs
    // based on VL53L0X_calc_timeout_mclks()
    int timeoutMicrosecondsToMclks(int timeout_period_us, int vcsel_period_pclks)
    {
        int macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

        return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
    }


    // based on VL53L0X_perform_single_ref_calibration()
    boolean performSingleRefCalibration(int vhv_init_byte)
    {
        writeReg(SYSRANGE_START, 0x01 | vhv_init_byte); // VL53L0X_REG_SYSRANGE_MODE_START_STOP

        startTimeout();
        while ((readReg(RESULT_INTERRUPT_STATUS) & 0x07) == 0)
        {
            if (checkTimeoutExpired()) { return false; }
        }

        writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);

        writeReg(SYSRANGE_START, 0x00);

        return true;
    }
    // Get the measurement timing budget in microseconds
    // based on VL53L0X_get_measurement_timing_budget_micro_seconds()
    // in us
    int getMeasurementTimingBudget()
    {
        AtomicReference<SequenceStepEnables> enables = new AtomicReference<>(new SequenceStepEnables());
        AtomicReference<SequenceStepTimeouts> timeouts = new AtomicReference<>(new SequenceStepTimeouts());

        int StartOverhead     = 1910; // note that this is different than the value in set_
        int EndOverhead        = 960;
        int MsrcOverhead       = 660;
        int TccOverhead        = 590;
        int DssOverhead        = 690;
        int PreRangeOverhead   = 660;
        int FinalRangeOverhead = 550;

        // "Start and end overhead times always present"
        int budget_us = StartOverhead + EndOverhead;

        getSequenceStepEnables(enables);
        getSequenceStepTimeouts(enables, timeouts);
        SequenceStepEnables a = enables.get();
        SequenceStepTimeouts b = timeouts.get();
        if (a.tcc)
        {
            budget_us += (b.msrc_dss_tcc_us + TccOverhead);
        }

        if (a.dss)
        {
            budget_us += 2 * (b.msrc_dss_tcc_us + DssOverhead);
        }
        else if (a.msrc)
        {
            budget_us += (b.msrc_dss_tcc_us + MsrcOverhead);
        }

        if (a.pre_range)
        {
            budget_us += (b.pre_range_us + PreRangeOverhead);
        }

        if (a.final_range)
        {
            budget_us += (b.final_range_us + FinalRangeOverhead);
        }

        measurement_timing_budget_us = budget_us; // store for internal reuse
        return budget_us;
    }

    enum vcselPeriodType { VcselPeriodPreRange, VcselPeriodFinalRange };

    int decodeVcselPeriod(int reg_val) {
        return ((reg_val + 1) << 1);
    }

    // Decode sequence step timeout in MCLKs from register value
    // based on VL53L0X_decode_timeout()
    // Note: the original function returned a uint32_t, but the return value is
    // always stored in a uint16_t.
    int decodeTimeout(int reg_val)
    {
        // format: "(LSByte * 2^MSByte) + 1"
        return (int)((reg_val & 0x00FF) <<
                (int)((reg_val & 0xFF00) >> 8)) + 1;
    }
    // Get the VCSEL pulse period in PCLKs for the given period type.
    // based on VL53L0X_get_vcsel_pulse_period()
    int getVcselPulsePeriod(vcselPeriodType type)
    {
        if (type == vcselPeriodType.VcselPeriodPreRange)
        {
            return decodeVcselPeriod(readReg(PRE_RANGE_CONFIG_VCSEL_PERIOD));
        }
        else if (type == vcselPeriodType.VcselPeriodFinalRange)
        {
            return decodeVcselPeriod(readReg(FINAL_RANGE_CONFIG_VCSEL_PERIOD));
        }
        else { return 255; }
    }
    // Get sequence step timeouts
    // based on get_sequence_step_timeout(),
    // but gets all timeouts instead of just the requested one, and also stores
    // intermediate values
    void getSequenceStepTimeouts(AtomicReference<SequenceStepEnables> enables, AtomicReference<SequenceStepTimeouts> timeouts)
    {
        SequenceStepTimeouts t = timeouts.get();
        SequenceStepEnables e = enables.get();
        t.pre_range_vcsel_period_pclks = getVcselPulsePeriod(vcselPeriodType.VcselPeriodPreRange);

        t.msrc_dss_tcc_mclks = readReg(MSRC_CONFIG_TIMEOUT_MACROP) + 1;
        t.msrc_dss_tcc_us =
                timeoutMclksToMicroseconds(t.msrc_dss_tcc_mclks,
                        t.pre_range_vcsel_period_pclks);

        t.pre_range_mclks =
                decodeTimeout(readReg16Bit(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
        t.pre_range_us =
                timeoutMclksToMicroseconds(t.pre_range_mclks,
                        t.pre_range_vcsel_period_pclks);

        t.final_range_vcsel_period_pclks = getVcselPulsePeriod(vcselPeriodType.VcselPeriodFinalRange);

        t.final_range_mclks =
                decodeTimeout(readReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

        if (e.pre_range)
        {
            t.final_range_mclks -= t.pre_range_mclks;
        }

        t.final_range_us =
                timeoutMclksToMicroseconds(t.final_range_mclks,
                        t.final_range_vcsel_period_pclks);

        enables.set(e);
        timeouts.set(t);
    }

    // Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs
    // based on VL53L0X_calc_timeout_us()
    int timeoutMclksToMicroseconds(int timeout_period_mclks, int vcsel_period_pclks)
    {
        int macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

        return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
    }

    int calcMacroPeriod(int vcsel_period_pclks) {
        return (((2304 * (vcsel_period_pclks) * 1655) + 500) / 1000);
    }



    public void startContinuous() {
        writeReg(0x80, 0x01);
        writeReg(0xFF, 0x01);
        writeReg(0x00, 0x00);
        writeReg(0x91, stop_variable);
        writeReg(0x00, 0x01);
        writeReg(0xFF, 0x00);
        writeReg(0x80, 0x00);
        writeReg(SYSRANGE_START, 0x02); // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
    }

    void stopContinuous()
    {
        writeReg(SYSRANGE_START, 0x01); // VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT

        writeReg(0xFF, 0x01);
        writeReg(0x00, 0x00);
        writeReg(0x91, 0x00);
        writeReg(0x00, 0x01);
        writeReg(0xFF, 0x00);
    }

    void startTimeout() {
        timeoutStartMs = (int)elapsedTime.milliseconds();
    }

    int readReg(int register) {
        int result = this.deviceClient.read8(register);
        return result;
    }

    int readReg16Bit(int register) {
        int value = 0;
        int upper = (this.deviceClient.read8(register)) << 8;
        int lower = this.deviceClient.read8(register + 0xff);
        value = (upper | lower);
        return value;
    }

    void readRangeContinuousMillimeters()
    {
        startTimeout();
        while ((readReg(RESULT_INTERRUPT_STATUS) & 0x07) == 0)
        {
            if (checkTimeoutExpired())
            {
                lastKnownRange = 65535;
            }
        }

        // assumptions: Linearity Corrective Gain is 1000 (default);
        // fractional ranging is not enabled
        int range = readReg16Bit(RESULT_RANGE_STATUS + 10);

        writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);

        lastKnownRange = range;
    }
    // Encode sequence step timeout register value from timeout in MCLKs
    // based on VL53L0X_encode_timeout()
    // Note: the original function took a uint16_t, but the argument passed to it
    // is always a uint16_t.
    int encodeTimeout(int timeout_mclks)
    {
        // format: "(LSByte * 2^MSByte) + 1"

        int ls_byte = 0;
        int ms_byte = 0;

        if (timeout_mclks > 0)
        {
            ls_byte = timeout_mclks - 1;

            while ((ls_byte & 0xFFFFFF00) > 0)
            {
                ls_byte >>= 1;
                ms_byte++;
            }

            return (ms_byte << 8) | (ls_byte & 0xFF);
        }
        else { return 0; }
    }
}
