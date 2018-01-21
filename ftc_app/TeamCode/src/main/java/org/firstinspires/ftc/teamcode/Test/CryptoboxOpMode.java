package org.firstinspires.ftc.teamcode.Test;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.*;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


import java.io.IOException;


@TeleOp(name="DogeCV Red Cryptobox Detector", group="DogeCV")

public class CryptoboxOpMode extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();


     private CryptoboxDetector cryptoboxDetector = null;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        telemetry.addData("Status", "Initialized");

        cryptoboxDetector = new CryptoboxDetector();
        cryptoboxDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 0, TeamColor.BLUE);

        //Optional Test Code to load images via Drawables
        //cryptoboxDetector.useImportedImage = true;
        //cryptoboxDetector.SetTestMat(com.qualcomm.ftcrobotcontroller.R.drawable.test_cv4);

        cryptoboxDetector.enable();


    }

    @Override
    public void init_loop() {
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
        runtime.reset();


    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        cryptoboxDetector.disable();
    }

}
