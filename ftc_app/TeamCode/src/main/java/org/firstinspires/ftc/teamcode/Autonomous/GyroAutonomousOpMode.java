/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Autonomous;

import android.graphics.Bitmap;
import android.util.Pair;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Shared.ClosableVuforiaLocalizer;
import org.firstinspires.ftc.teamcode.Shared.Direction;
import org.firstinspires.ftc.teamcode.Shared.FourWheelMecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Shared.RobotHardware;
import org.firstinspires.ftc.teamcode.Test.CryptoboxDetector;
import org.firstinspires.ftc.teamcode.Test.TeamColor;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;

import static java.lang.Math.sin;
import static java.util.Collections.max;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name = "Gyro Autonomous", group = "Linear Opmode")
public class GyroAutonomousOpMode extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    final int maxBufferResults = 25;

    // Used to access robot hardware
    RobotHardware hw;

    FourWheelMecanumDrivetrain drivetrain;

    // Stores instance of Vuforia Localizer
    ClosableVuforiaLocalizer vuforia;


    final double turnThreshold = 2;

    JewelDetector jewels = new JewelDetector();

    // Instance of relic trackable
    VuforiaTrackable relicTemplate;

    RelicRecoveryVuMark lastKnownVumark = RelicRecoveryVuMark.UNKNOWN;
    JewelDetector.JewelOrder order;

    LinkedList<Pair<RelicRecoveryVuMark, JewelDetector.JewelOrder>> resultsBuffer = new LinkedList<>();
    BNO055IMU imu;

    public void ReinitializeIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public void GyroTurn(double speed, double angle) {
        // Angle is counterclockwise (sorry)

        double normalizedHeading = normalize(getHeading());
        double normalizedAngle = normalize(angle);
        double angleDiff = normalizedHeading - normalizedAngle;

        angleDiff = (angleDiff / 180) * Math.PI;
        double c = sin(angleDiff);
        if (c >= 0) {
            // CW
            drivetrain.MoveAngle(0, 0, speed);

        }
        else if (c < 0) {
            // CCW
            drivetrain.MoveAngle(0, 0, -speed);
        }

        while (opModeIsActive()) {
            double angle1 = normalize(angle + turnThreshold);
            double angle2 = normalize(angle - turnThreshold);
            double target = normalize(getHeading());
            double diff = normalize(angle2 - angle1);
            if (diff > 180) {
                double temp = angle1;
                angle1 = angle2;
                angle2 = temp;
            }
            boolean within = false;
            if (angle1 <= angle2) {
                within = target >= angle1 && target <= angle2;
            }

            else {
                within = target >= angle1 || target <= angle2;
            }

            if (within) {
                drivetrain.stop();
                break;
            }
        }
    }
    double normalize(double angle) {
        angle = (360 + angle % 360) % 360;
        return angle;
    }

    double getHeading() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
    @Override
    public void runOpMode() throws InterruptedException {
        hw = RobotHardware.GetSingleton(hardwareMap);
        RobotHardware.SetCurrentRunningOpMode(this);
        String mode = "";
        MatchParameters parameters = MatchParameters.current;
        if (parameters == null) {
            requestOpModeStop();
        }
        try {
            mode = parameters.get("start");
        } catch (Exception e) {
            requestOpModeStop();
        }
        boolean close = mode.contains("CLOSE");

        boolean red = mode.contains("RED");
        drivetrain = new FourWheelMecanumDrivetrain();
        drivetrain.setRunningOpMode(this);
        hw.linearSlideDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hw.forwardRight.setDirection(DcMotorSimple.Direction.REVERSE);
        hw.backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        drivetrain.setMotorZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);
        drivetrain.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        resetJewelArms();

        if (imu == null) {
            ReinitializeIMU();
        }
        imu.startAccelerationIntegration(new Position(), new Velocity(), 16);
        resetJewelArms();

        //region Vuforia/Vision
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Jewel Detector Settings
        jewels.areaWeight = 0.02;
        jewels.detectionMode = JewelDetector.JewelDetectionMode.MAX_AREA; // PERFECT_AREA
        //jewels.perfectArea = 6500; <- Needed for PERFECT_AREA
        jewels.debugContours = true;
        jewels.maxDiffrence = 15;
        jewels.ratioWeight = 15;
        jewels.minArea = 700;


        if (red) {
            hw.phoneServo2.setPosition(0.67); // 0.64
        } else {
            hw.phoneServo1.setPosition(0.16);  // 0.27
        }

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters vuforiaParameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        vuforiaParameters.vuforiaLicenseKey = "ASK0CtX/////AAAAGbyNvvxXMUrohVJwXgMBW5Iqmz//UVeASb2KC//wHTXPuoN/gOM0vbw91nX++j+iS98pzeEfO+p9jpijt7j6VgQZlFTO9K2HjwAvTmG5M6CYglrh0B3kfA/nZx/NSyyxWIRe7Q03DeNDH50ZnSJ3I4FkyD7AbcTbJHg3LjL72N6/Lfm5biUbhOPoeQb1a8qUaqp1Il340pGFEvIEH8s7nhqHAga3TSdvM7yWxqRtZ3Bv2yEmIFMIWuBdEV6ahooWDsnnRmSn33bQVRD+KTNNocJWkwQ1pDG/8XBzswiICKBvcCvzklZhV/TeoJDk6tagk8sMcYxBdzMtyBlYVA9y3iJRyuzu+UMfswmSlA6CnmM/";

        vuforiaParameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = new ClosableVuforiaLocalizer(vuforiaParameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        relicTrackables.activate();
        //endregion

        hw.altClawTurn.setPosition(0.5);  // center
        hw.upperLeft.setPosition(0.05);
        hw.upperRight.setPosition(0.90);
        // Wait for the game to start (driver presses PLAY)
        waitForStart(red);

        // TODO: make vuforia close during stop so drivers can safely restart if necessary
        relicTrackables.deactivate();
        vuforia.close();

        resetFlickers();

        hw.upperLeft.setPosition(0.19);
        hw.upperRight.setPosition(0.77);

        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            Direction vumark = null;
            switch (lastKnownVumark) {
                case LEFT:
                    vumark = Direction.LEFT;
                    break;
                case RIGHT:
                    vumark = Direction.RIGHT;
                    break;
                case CENTER:
                    vumark = Direction.FORWARD;
                    break;
                default:
                    vumark = Direction.FORWARD;
                    break;
            }

            jewelArms(!red);



            hw.linearSlideDriveMotor.setPower(-0.75);
            sleep(1200);
            hw.linearSlideDriveMotor.setPower(0);

            // Open altClaw
            hw.altClawLeft.setPosition(0.29);  // 0.225
            hw.altClawRight.setPosition(0.65);  // 0.727

            flick(!red, order);
            sleep(500);

            hw.linearSlideDriveMotor.setPower(0.75);
            sleep(750);
            hw.linearSlideDriveMotor.setPower(0);

            resetJewelArms();
            resetFlickers();

            // Close altClaw
            hw.altClawLeft.setPosition(0.67);  //0.610
            hw.altClawRight.setPosition(0.30);  //0.276
            sleep(500);

            hw.linearSlideDriveMotor.setPower(-0.75);
            sleep(1000);
            hw.linearSlideDriveMotor.setPower(0);

            if (close) {
                if (red) {
                    switch (vumark) {
                        case RIGHT:
                            AutoMove(0.25, 0, 1240);
                            break;
                        case FORWARD:
                            AutoMove(0.25, 0, 1572);
                            break;
                        case LEFT:
                            AutoMove(0.25, 0, 1904);
                            break;
                    }
                    GyroTurn(0.15, -90);
                } else {
                    switch (vumark) {
                        case LEFT:
                            AutoMove(0.25, 0, 1203);
                            break;
                        case FORWARD:
                            AutoMove(0.25, 0, 1547);
                            break;
                        case RIGHT:
                            AutoMove(0.25, 0, 1889);
                            break;
                    }
                    GyroTurn(0.15, 90);
                }
            } else {
                AutoMove(0.25, 0, 1050);

                if (red) {
                    /*moveHorizontal(0.15, 0.0003, 300, Direction.LEFT);
                    visionCrypto(0.1, 0.0003, Direction.LEFT, vumark);*/
                    switch (vumark) {
                        case LEFT:
                            moveHorizontal(0.1, 0.0003, 1075, Direction.LEFT);
                            break;
                        case FORWARD:
                            moveHorizontal(0.1, 0.0003, 635, Direction.LEFT);
                            break;
                        case RIGHT:
                            moveHorizontal(0.1, 0.0003, 280, Direction.LEFT);
                            break;
                    }
                } else {
                    switch (vumark) {
                        case LEFT:
                            moveHorizontal(0.1, 0.0003, 180, Direction.RIGHT);
                            break;
                        case FORWARD:
                            moveHorizontal(0.1, 0.0003, 615, Direction.RIGHT);
                            break;
                        case RIGHT:
                            moveHorizontal(0.1, 0.0003, 1035, Direction.RIGHT);
                            break;
                    }
                }
            }

            AutoMove(0.25, 0, 325);
            AutoMove(-0.25, 0, 25);

            hw.linearSlideDriveMotor.setPower(0.75);
            sleep(500);
            hw.linearSlideDriveMotor.setPower(0);

            release();
            AutoMove(-0.5, 0, 200);
            if (vumark != Direction.FORWARD) {
                AutoMove(0.2, vumark == Direction.RIGHT ? -90 : 90, 175);
            }

            hw.altClawLeft.setPosition(0.88);
            hw.altClawRight.setPosition(0.12);

            if (close) {
                if (red) {
                    GyroTurn(0.4, 90);
                } else {
                    GyroTurn(0.4, -90);
                }
            } else {
                if (red) {
                    GyroTurn(0.4, 180);
                } else {
                    GyroTurn(0.4, 180);
                }
            }
            AutoMoveByTime(-0.8, 0, 370, 2000);
            AutoMove(0.8, 0, 100);

            release();
            hw.upperLeft.setPosition(0.05);
            hw.upperRight.setPosition(0.90);

            requestOpModeStop();
        }
    }

    public void AutoMove(double speed, double angle, int counts) {
        int initialForward = hw.forwardLeft.getCurrentPosition();
        int initialBackward = hw.backLeft.getCurrentPosition();

        drivetrain.MoveAngle(speed, angle, 0);

        while (opModeIsActive()) {
            int differenceForward = Math.abs(hw.forwardLeft.getCurrentPosition() - initialForward);
            int differenceBackward = Math.abs(hw.backLeft.getCurrentPosition() - initialBackward);
            telemetry.addData("d1", differenceForward);
            telemetry.addData("d2", differenceBackward);
            telemetry.update();
            if ((differenceBackward + differenceForward) / 2 > counts) {
                drivetrain.stop();
                break;
            }
        }
    }

    public void AutoMoveByTime(double speed, double angle, int counts, double timeout) {
        double start = runtime.milliseconds();
        int initialForward = hw.forwardLeft.getCurrentPosition();
        int initialBackward = hw.backLeft.getCurrentPosition();

        drivetrain.MoveAngle(speed, angle, 0);

        while (opModeIsActive() && runtime.milliseconds() - start < timeout) {
            int differenceForward = Math.abs(hw.forwardLeft.getCurrentPosition() - initialForward);
            int differenceBackward = Math.abs(hw.backLeft.getCurrentPosition() - initialBackward);
            telemetry.addData("d1", differenceForward);
            telemetry.addData("d2", differenceBackward);
            telemetry.update();
            if ((differenceBackward + differenceForward) / 2 > counts) {
                drivetrain.stop();
                break;
            }
        }
    }

    public void flick(boolean left, JewelDetector.JewelOrder order) {

        if (left) {
            if (order == JewelDetector.JewelOrder.RED_BLUE) {
                hw.leftFlick.setPosition(1); // Flick back
            } else {
                hw.leftFlick.setPosition(0); // Flick forward
            }
        } else {
            if (order == JewelDetector.JewelOrder.RED_BLUE) {
                // Flick back
                hw.rightFlick.setPosition(0);
            } else {
                // Flick forward
                hw.rightFlick.setPosition(1);
            }
        }
    }

    void resetFlickers() {
        hw.leftFlick.setPosition(0.42);
        hw.rightFlick.setPosition(0.50);
    }

    private void moveHorizontal(double speed, double p, double amount, Direction direction) {
        int backLeftStart = hw.backLeft.getCurrentPosition();
        int backRightStart = hw.backRight.getCurrentPosition();
        int forwardLeftStart = hw.forwardLeft.getCurrentPosition();
        int forwardRightStart = hw.forwardRight.getCurrentPosition();

        while (opModeIsActive()) {
            int backLeft = hw.backLeft.getCurrentPosition();
            int backRight = hw.backRight.getCurrentPosition();
            int forwardLeft = hw.forwardLeft.getCurrentPosition();
            int forwardRight = hw.forwardRight.getCurrentPosition();

            int backLeftDiff = Math.abs(backLeft - backLeftStart);
            int backRightDiff = Math.abs(backRight - backRightStart);
            int forwardLeftDiff = Math.abs(forwardLeft - forwardLeftStart);
            int forwardRightDiff = Math.abs(forwardRight - forwardRightStart);

            double avg = (backLeftDiff + backRightDiff + forwardLeftDiff + forwardRightDiff) / 4;
            double backLeftComp = (avg - backLeftDiff) * p;
            double backRightComp = (avg - backRightDiff) * p;
            double forwardLeftComp = (avg - forwardLeftDiff) * p;
            double forwardRightComp = (avg - forwardRightDiff) * p;

            if (avg >= amount) {
                break;
            }
            switch (direction) {
                case LEFT:
                    hw.forwardRight.setPower(speed + forwardRightComp);
                    hw.forwardLeft.setPower(-speed - forwardLeftComp);
                    hw.backRight.setPower(-speed - backRightComp);
                    hw.backLeft.setPower(speed + backLeftComp);
                    break;
                case RIGHT:
                    hw.forwardRight.setPower(-speed - forwardRightComp);
                    hw.forwardLeft.setPower(speed + forwardLeftComp);
                    hw.backRight.setPower(speed + backRightComp);
                    hw.backLeft.setPower(-speed - backLeftComp);
                    break;

            }
        }

        drivetrain.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drivetrain.stop();
    }

    private void release() {
        hw.altClawLeft.setPosition(0.29);
        hw.altClawRight.setPosition(0.65);
    }

    private void jewelArms(boolean left) {
        if (left) {
            hw.jewelArm1.setPosition(0.76);  // 0.511
        } else {
            hw.jewelArm2.setPosition(0.34); // 0.349
        }
    }

    private void resetJewelArms() {
        hw.jewelArm1.setPosition(0.18);
        hw.jewelArm2.setPosition(0.90);
    }

    public void waitForStart(boolean red) {
        Image rgb = null;
        Mat tmp = new Mat();
        Mat tmpGray = new Mat();
        vuforia.setFrameQueueCapacity(10);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        while (!isStarted()) {
            try {
                VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take();
                long numImages = frame.getNumImages();
                for (int i = 0; i < numImages; i++) {
                    if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                        rgb = frame.getImage(i);
                        break;
                    }
                }
                if (rgb != null) {
                    Bitmap bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
                    bm.copyPixelsFromBuffer(rgb.getPixels());
                    tmp = new Mat(rgb.getWidth(), rgb.getHeight(), CvType.CV_8UC3);

                    Utils.bitmapToMat(bm, tmp);
                    Imgproc.cvtColor(tmp, tmpGray, Imgproc.COLOR_RGB2GRAY);
                    jewels.processFrame(tmp, tmpGray);
                    JewelDetector.JewelOrder order = jewels.getCurrentOrder();
                    if (red) {
                        if (order == JewelDetector.JewelOrder.BLUE_RED) {
                            order = JewelDetector.JewelOrder.RED_BLUE;
                        }
                        else {
                            order = JewelDetector.JewelOrder.BLUE_RED;
                        }
                    }
                    RelicRecoveryVuMark vm = RelicRecoveryVuMark.from(relicTemplate);
                    resultsBuffer.add(new Pair<>(vm, order));
                    if (resultsBuffer.size() > maxBufferResults) {
                        resultsBuffer.removeFirst();
                    }
                    Results results = analyzeBuffer();
                    telemetry.addData("Vision", results.toString());
                    telemetry.update();
                    lastKnownVumark = vm;
                    this.order = order;
                    frame.close();
                }

            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return;
            }
        }
    }

    class Results {
        public Map<RelicRecoveryVuMark, Integer> vumarkOccurences = new HashMap<>();
        public Map<JewelDetector.JewelOrder, Integer> jewelOccurences = new HashMap<>();
        RelicRecoveryVuMark mostCommonVuMark = RelicRecoveryVuMark.UNKNOWN;
        int vF;
        JewelDetector.JewelOrder mostCommonOrder = JewelDetector.JewelOrder.UNKNOWN;
        int oF;

        public Results() {
            vumarkOccurences.put(RelicRecoveryVuMark.LEFT, 0);
            vumarkOccurences.put(RelicRecoveryVuMark.CENTER, 0);
            vumarkOccurences.put(RelicRecoveryVuMark.RIGHT, 0);
            vumarkOccurences.put(RelicRecoveryVuMark.UNKNOWN, 0);

            jewelOccurences.put(JewelDetector.JewelOrder.BLUE_RED, 0);
            jewelOccurences.put(JewelDetector.JewelOrder.RED_BLUE, 0);
            jewelOccurences.put(JewelDetector.JewelOrder.UNKNOWN, 0);
        }

        @Override
        public String toString() {
            return "Most likely VuMark: " + mostCommonVuMark.toString() + "\n" +
                    "Certainty: " + (vF / maxBufferResults) * 100 + "%" + "\n" +
                    "Most likely Jewel Order: " + mostCommonOrder + "\n" +
                    "Certainty: " + (oF / maxBufferResults) * 100 + "%";
        }
    }

    public Results analyzeBuffer() {
        Results results = new Results();
        for (Pair<RelicRecoveryVuMark, JewelDetector.JewelOrder> pair : resultsBuffer) {
            RelicRecoveryVuMark vm = pair.first;
            JewelDetector.JewelOrder order = pair.second;

            results.vumarkOccurences.put(vm, results.vumarkOccurences.get(vm) + 1);
            results.jewelOccurences.put(order, results.jewelOccurences.get(order) + 1);
        }

        int maxVuMarkFrequency = Collections.max(results.vumarkOccurences.values());
        for (Map.Entry<RelicRecoveryVuMark, Integer> vuMarkIntegerEntry : results.vumarkOccurences.entrySet()) {
            if (vuMarkIntegerEntry.getValue() == maxVuMarkFrequency) {
                results.mostCommonVuMark = vuMarkIntegerEntry.getKey();
            }
        }

        results.vF = maxVuMarkFrequency;

        int maxOrderFrequency = Collections.max(results.jewelOccurences.values());
        for (Map.Entry<JewelDetector.JewelOrder, Integer> jewelIntegerEntry : results.jewelOccurences.entrySet()) {
            if (jewelIntegerEntry.getValue() == maxOrderFrequency) {
                results.mostCommonOrder = jewelIntegerEntry.getKey();
            }
        }

        results.oF = maxOrderFrequency;
        return results;
    }

}