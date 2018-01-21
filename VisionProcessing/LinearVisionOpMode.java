/*
 * Copyright (c) 2016 Arthur Pachachura, LASA Robotics, and contributors
 * MIT licensed
 *
 * Some code from FIRST library, Copyright (C) Qualcomm
 *
 * Thank you to Russell Coleman (LASA).
 */
package org.firstinspires.ftc.teamcode.Shared.VisionProcessing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.TimestampedI2cData;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryInternal;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import java.util.concurrent.CancellationException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.TimeUnit;

/**
 * Linear version of the Vision OpMode
 * This includes code from the FIRST library (C) Qualcomm as of 2016/10/6
 */
public abstract class LinearVisionOpMode extends VisionOpModeCore {

    //------------------------------------------------------------------------------------------------
    // State
    //------------------------------------------------------------------------------------------------

    private LinearVisionOpMode.LinearOpModeHelper helper          = null;
    private ExecutorService executorService = null;
    private volatile boolean   isStarted       = false;
    private volatile boolean   stopRequested   = false;

    private Mat rgba;
    private Mat gray;
    private boolean hasNewFrame = false;

    public LinearVisionOpMode() {
    }

    @Override
    public final Mat frame(Mat rgba, Mat gray) {
        if (!isStarted) return rgba;
        Imgproc.cvtColor(rgba, this.gray, Imgproc.COLOR_RGBA2GRAY);
        hasNewFrame = true;
        return rgba;
    }

    public final Mat getFrameRgba() {
        return rgba;
    }

    public final Mat getFrameGray() {
        return gray;
    }

    public boolean hasNewFrame() {
        return hasNewFrame;
    }

    public void discardFrame() {
        hasNewFrame = false;
    }

    public abstract void runOpMode() throws InterruptedException;

    public final void waitForVisionStart() throws InterruptedException {
        //Give some status info
        //telemetry.addData("Vision Status", "Initializing...\r\n" +
          //      "Please wait, do not stop the OpMode.");

        while (!this.isInitialized()) {
            synchronized (this) {
                this.wait();
            }
        }
    }

    public synchronized void waitForStart() throws InterruptedException {
        while (!isStarted()) {
            synchronized (this) {
                try {
                    this.wait();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return;
                }
            }
        }
    }

    public void waitOneFullHardwareCycle() throws InterruptedException {
        this.waitForNextHardwareCycle();
        Thread.sleep(1L);
        this.waitForNextHardwareCycle();
    }

    private void waitForNextHardwareCycle() throws InterruptedException {
        synchronized (this) {
            this.wait();
        }
    }

    public final void idle() {
        // Otherwise, yield back our thread scheduling quantum and give other threads at
        // our priority level a chance to run
        Thread.yield();
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public final boolean opModeIsActive() {
        boolean isActive = !this.isStopRequested() && this.isStarted();
        if (isActive) {
            idle();
        }
        return isActive;
    }

    public final boolean isStarted() {
        return this.isStarted || Thread.currentThread().isInterrupted();
    }

    public final boolean isStopRequested() {
        return this.stopRequested || Thread.currentThread().isInterrupted();
    }



    @Override
    public final void init() {
        super.init();
        hasNewFrame = false;
        this.rgba = new Mat(height, width, CvType.CV_8UC4);
        this.gray = new Mat(height, width, CvType.CV_8UC1);

        this.executorService = ThreadPool.newSingleThreadExecutor("LinearOpMode");
        this.helper          = new LinearVisionOpMode.LinearOpModeHelper();
        this.isStarted       = false;
        this.stopRequested   = false;

        this.executorService.execute(helper);
    }

    @Override
    public final void init_loop() {
        handleLoop();
    }

    @Override
    final public void start() {
        stopRequested = false;
        isStarted = true;
        synchronized (this) {
            this.notifyAll();
        }
    }

    @Override
    public final void loop() {
        super.loop();
        handleLoop();
    }

    @Override
    public final void stop() {
        super.stop();
        this.rgba.release();
        this.gray.release();

        // make isStopRequested() return true (and opModeIsActive() return false)
        stopRequested = true;

        if (executorService != null) {  // paranoia

            // interrupt the linear opMode and shutdown it's service thread
            executorService.shutdownNow();

            /** Wait, forever, for the OpMode to stop. If this takes too long, then
             * {@link OpModeManagerImpl#callActiveOpModeStop()} will catch that and take action */
            try {
                String serviceName = "user linear op mode";
                ThreadPool.awaitTermination(executorService, 100, TimeUnit.DAYS, serviceName);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }

    protected void handleLoop() {
        // if there is a runtime exception in user code; throw it so the normal error
        // reporting process can handle it
        if (helper.hasRuntimeException()) {
            throw helper.getRuntimeException();
        }

        synchronized (this) {
            this.notifyAll();
        }
    }

    protected class LinearOpModeHelper implements Runnable {

        protected RuntimeException exception  = null;
        protected boolean          isShutdown = false;

        public LinearOpModeHelper() {
        }

        @Override
        public void run() {
            ThreadPool.logThreadLifeCycle("LinearOpMode main", new Runnable() { @Override public void run() {
                exception = null;
                isShutdown = false;

                try {
                    LinearVisionOpMode.this.runOpMode();
                    requestOpModeStop();
                } catch (InterruptedException ie) {
                    // InterruptedException, shutting down the op mode
                    RobotLog.d("LinearOpMode received an InterruptedException; shutting down this linear op mode");
                } catch (CancellationException ie) {
                    // In our system, CancellationExceptions are thrown when data was trying to be acquired, but
                    // an interrupt occurred, and you're in the unfortunate situation that the data acquisition API
                    // involved doesn't allow InterruptedExceptions to be thrown. You can't return (what data would
                    // you return?), and so you have to throw a RuntimeException. CancellationException seems the
                    // best choice.
                    RobotLog.d("LinearOpMode received a CancellationException; shutting down this linear op mode");
                } catch (RuntimeException e) {
                    exception = e;
                } finally {
                    // If the user has given us a telemetry.update() that hasn't get gone out, then
                    // push it out now. However, any NEW device health warning should be suppressed while
                    // doing so, since required state might have been cleaned up by now and thus generate errors.
                    TimestampedI2cData.suppressNewHealthWarningsWhile(new Runnable() {
                        @Override public void run() {
                            if (telemetry instanceof TelemetryInternal) {
                                telemetry.setMsTransmissionInterval(0); // will be reset the next time the opmode runs
                                ((TelemetryInternal) telemetry).tryUpdateIfDirty();
                            }
                        }
                    });
                    // Do the necessary bookkeeping
                    isShutdown = true;
                }
            }});
        }

        public boolean hasRuntimeException() {
            return (exception != null);
        }

        public RuntimeException getRuntimeException() {
            return exception;
        }

        public boolean isShutdown() {
            return isShutdown;
        }
    }

    //----------------------------------------------------------------------------------------------
    // Telemetry management
    //----------------------------------------------------------------------------------------------

    @Override public void internalPostInitLoop() {
        // Do NOT call super, as that updates telemetry unilaterally
        if (telemetry instanceof TelemetryInternal) {
            ((TelemetryInternal)telemetry).tryUpdateIfDirty();
        }
    }

    @Override public void internalPostLoop() {
        // Do NOT call super, as that updates telemetry unilaterally
        if (telemetry instanceof TelemetryInternal) {
            ((TelemetryInternal)telemetry).tryUpdateIfDirty();
        }
    }
}
