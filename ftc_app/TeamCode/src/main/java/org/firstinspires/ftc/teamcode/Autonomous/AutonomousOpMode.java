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

import com.qualcomm.hardware.adafruit.AdafruitI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Shared.Direction;
import org.firstinspires.ftc.teamcode.Shared.FourWheelMecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Shared.RobotHardware;
import org.firstinspires.ftc.teamcode.Shared.VL53L0X;
import org.opencv.core.Mat;


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

@Autonomous(name = "Autonomous", group = "Linear Opmode")
public class AutonomousOpMode extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // Used to access robot hardware
    RobotHardware hw;

    FourWheelMecanumDrivetrain drivetrain;

    // Stores instance of Vuforia Localizer
    VuforiaLocalizer vuforia;

    // Instance of relic trackable
    VuforiaTrackable relicTemplate;

    RelicRecoveryVuMark lastKnownVumark = RelicRecoveryVuMark.UNKNOWN;

    double distanceThreshold;

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
        }
        catch (Exception e) {
            requestOpModeStop();
        }
        boolean close = mode.contains("CLOSE");

        boolean red = mode.contains("RED");
        drivetrain = new FourWheelMecanumDrivetrain();
        hw.linearSlideDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hw.forwardRight.setDirection(DcMotorSimple.Direction.REVERSE);
        hw.backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        drivetrain.setMotorZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);
        drivetrain.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (red) {
            hw.phoneServo2.setPosition(0.62);
        }
        else {
            hw.phoneServo1.setPosition(0.1);
        }
        resetJewelArms();

        // MatchParameters parameters = MatchParameters.loadParameters(FtcRobotControllerActivity.matchParameterData);
        if (hw.imu == null) {
            hw.ReinitializeIMU();
        }
        hw.imu.startAccelerationIntegration(new Position(), new Velocity(), 16);
        resetJewelArms();
        resetFlickers();

        //region Vuforia
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters vuforiaParameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        vuforiaParameters.vuforiaLicenseKey = "ASK0CtX/////AAAAGbyNvvxXMUrohVJwXgMBW5Iqmz//UVeASb2KC//wHTXPuoN/gOM0vbw91nX++j+iS98pzeEfO+p9jpijt7j6VgQZlFTO9K2HjwAvTmG5M6CYglrh0B3kfA/nZx/NSyyxWIRe7Q03DeNDH50ZnSJ3I4FkyD7AbcTbJHg3LjL72N6/Lfm5biUbhOPoeQb1a8qUaqp1Il340pGFEvIEH8s7nhqHAga3TSdvM7yWxqRtZ3Bv2yEmIFMIWuBdEV6ahooWDsnnRmSn33bQVRD+KTNNocJWkwQ1pDG/8XBzswiICKBvcCvzklZhV/TeoJDk6tagk8sMcYxBdzMtyBlYVA9y3iJRyuzu+UMfswmSlA6CnmM/";

        vuforiaParameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(vuforiaParameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        relicTrackables.activate();
        //endregion

        Runnable motorLift = new Runnable() {
            @Override
            public void run() {
                try {
                    hw.linearSlideDriveMotor.setPower(-0.75);
                    Thread.sleep(1200);
                    hw.linearSlideDriveMotor.setPower(0);
                } catch (InterruptedException e) {

                }
            }
        };
        Runnable motorDrop = new Runnable() {
            @Override
            public void run() {
                try {
                    hw.linearSlideDriveMotor.setPower(0.75);
                    Thread.sleep(1000);
                    hw.linearSlideDriveMotor.setPower(0);
                } catch (InterruptedException e) {

                }
            }
        };

        Thread liftGlyph = new Thread(motorLift);
        Thread dropGlpyh = new Thread(motorDrop);

        hw.right_color.enableLed(true);
        hw.altClawTurn.setPosition(0.5);  // center
        hw.upperLeft.setPosition(0.03);
        hw.upperRight.setPosition(0.89);

        // Wait for the game to start (driver presses PLAY)
        this.waitForStart(red, close);
        hw.upperLeft.setPosition(0.23);
        hw.upperRight.setPosition(0.77);

        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            Direction vumark = null;
            switch(lastKnownVumark) {
                case LEFT:
                    vumark = Direction.LEFT;
                    break;
                case RIGHT:
                    vumark = Direction.RIGHT;
                    break;
                case CENTER:
                    vumark = Direction.FORWARD;
                    break;
            }

            jewelArms(!red);

            hw.linearSlideDriveMotor.setPower(-0.75);
            sleep(500);
            hw.linearSlideDriveMotor.setPower(0);

            // Open altClaw
            hw.altClawLeft.setPosition(0.29);  // 0.225
            hw.altClawRight.setPosition(0.65);  // 0.727

            sleep(1000);

            flick(!red, red);

            dropGlpyh.start();

            sleep(1000);

            resetJewelArms();
            resetFlickers();

            // Close altClaw
            hw.altClawLeft.setPosition(0.67);  //0.610
            hw.altClawRight.setPosition(0.30);  //0.276
            sleep(500);
            liftGlyph.start();
            sleep(1200);
            if (close) {
                AutoMove(0.25, 0, 1050);
                if (red) {
                    //AutoMove(0.1, 0, 100);
                    drivetrain.GyroTurn(0.15, -75);
                }
                else {
                    //AutoMove(0.1, 0, 100);
                    drivetrain.GyroTurn(0.15, 90);
                }
                AutoMove(0.15, 0, 20);
            }
            else {
                AutoMove(0.25, 0, 1080);
                sleep(300);
            }
            if (red) {
                lightCrypto(0.1, 0.0003, Direction.LEFT, vumark, !red, 425);
            }
            else {
                lightCrypto(0.1, 0.0003, Direction.RIGHT, vumark, !red, 425);
            }

            AutoMove(0.25, 0, 255);
            AutoMove(-0.5, 0, 25);

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
                    drivetrain.GyroTurn(0.4, 90);
                }
                else {
                    drivetrain.GyroTurn(0.4, -90);
                }
            }
            else
            {
                if (red) {
                    drivetrain.GyroTurn(0.4, -180);
                }
                else {
                    drivetrain.GyroTurn(0.4, 180);
                }
            }
            AutoMoveByTime(-0.8, 0, 325, 2000);
            AutoMove(0.8, 0, 100);

            release();
            hw.upperLeft.setPosition(0.03);
            hw.upperRight.setPosition(0.89);

            requestOpModeStop();
        }

        hw.imu.stopAccelerationIntegration();

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

    public void flick(boolean left, boolean red) {
        if (left) {
            if (hw.left_color.red() > hw.left_color.blue() * 1.25) {
                if (red) {
                    // Flick forward
                    hw.leftFlick.setPosition(0);
                }
                else {
                    // Flick back
                    hw.leftFlick.setPosition(1);
                }
            }
            else {
                if (red) {
                    // Flick back
                    hw.leftFlick.setPosition(1);
                }
                else {
                    // Flick forward
                    hw.leftFlick.setPosition(0);
                }
            }
        }
        else {
            if (hw.right_color.red() > hw.right_color.blue() * 1.25) {
                if (red) {
                    // Flick back
                    hw.rightFlick.setPosition(1);
                }
                else {
                    // Flick forward
                    hw.rightFlick.setPosition(0);
                }
            }
            else {
                if (red) {
                    // Flick forward
                    hw.rightFlick.setPosition(0);
                }
                else {
                    // Flick back
                    hw.rightFlick.setPosition(1);
                }
            }
        }
    }

    void resetFlickers() {
        hw.leftFlick.setPosition(0.38);
        hw.rightFlick.setPosition(0.50);
    }

    class Encoders {
        int backLeft;
        int backRight;
        int forwardLeft;
        int forwardRight;

        public Encoders() {

        }
        public Encoders(RobotHardware robot) {
            this.backLeft = robot.backLeft.getCurrentPosition();
            this.backRight = robot.backRight.getCurrentPosition();
            this.forwardLeft = robot.forwardLeft.getCurrentPosition();
            this.forwardRight = robot.forwardRight.getCurrentPosition();
        }

        public void set(FourWheelMecanumDrivetrain dt, RobotHardware hw) {
            dt.setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
            hw.backLeft.setTargetPosition(backLeft);
            hw.backRight.setTargetPosition(backRight);
            hw.forwardLeft.setTargetPosition(forwardLeft);
            hw.forwardRight.setTargetPosition(forwardRight);
        }

        public Encoders average(Encoders other) {
            Encoders e = new Encoders();
            e.backLeft = (this.backLeft + other.backLeft) / 2;
            e.backRight = (this.backRight + other.backRight) / 2;
            e.forwardLeft = (this.forwardLeft + other.forwardLeft) / 2;
            e.forwardRight = (this.forwardRight + other.forwardRight) / 2;
            return e;
        }

        public void applyOffset(Direction direction, double amount) {
            if (direction == Direction.FORWARD) {
                return;
            }
            if (direction == Direction.LEFT) {
                forwardLeft -= amount;
                forwardRight += amount;
                backLeft += amount;
                backRight -= amount;
            }
            if (direction == Direction.RIGHT) {
                forwardLeft += amount;
                forwardRight -= amount;
                backLeft -= amount;
                backRight += amount;
            }
        }

        public int totalDiff(Encoders a) {
            return Math.abs(a.backLeft - backLeft) + Math.abs(a.backRight - backRight) + Math.abs(a.forwardLeft - forwardLeft) + Math.abs(a.forwardRight - forwardRight);
        }
    }

    private Direction opposite(Direction input) {
        if (input == Direction.BACKWARD) {
            return Direction.FORWARD;
        }
        if (input == Direction.FORWARD) {
            return Direction.BACKWARD;
        }
        if (input == Direction.LEFT) {
            return Direction.RIGHT;
        }
        if (input == Direction.RIGHT) {
            return Direction.LEFT;
        }
        return null;
    }

    private void lightCrypto(double speed, double p, Direction direction, Direction desired, boolean blue, int offset) {
        int backLeftStart = hw.backLeft.getCurrentPosition();
        int backRightStart = hw.backRight.getCurrentPosition();
        int forwardLeftStart = hw.forwardLeft.getCurrentPosition();
        int forwardRightStart = hw.forwardRight.getCurrentPosition();
        int count = 0;
        double firstTime = 0;
        double adjustedOffset = offset;
        if (opposite(direction) == desired) {
            adjustedOffset += 25;
            RobotLog.i("opp dir");
        }

        Encoders first = null;
        Encoders second = null;
        Encoders middle = null;

        double dominantColor;
        double secondaryColor;
        double multiplier;

        while (opModeIsActive()) {
            double timeElapsed = runtime.milliseconds() - firstTime;
            if (blue) {
                dominantColor = hw.bottom_color.blue();
                secondaryColor = hw.bottom_color.red();
                multiplier = 1.25;
            }
            else {
                dominantColor = hw.bottom_color.red();
                secondaryColor = hw.bottom_color.blue();
                multiplier = 1.25;
            }

            if (dominantColor > (secondaryColor * multiplier)) {

                first = new Encoders(hw);
                while (opModeIsActive() && dominantColor > (secondaryColor * multiplier)) {
                    if (blue) {
                        dominantColor = hw.bottom_color.blue();
                        secondaryColor = hw.bottom_color.red();
                    }
                    else {
                        dominantColor = hw.bottom_color.red();
                        secondaryColor = hw.bottom_color.blue();
                    }
                }
                while (opModeIsActive() && !(dominantColor > (secondaryColor * multiplier))){
                    if (blue) {
                        dominantColor = hw.bottom_color.blue();
                        secondaryColor = hw.bottom_color.red();
                    }
                    else {
                        dominantColor = hw.bottom_color.red();
                        secondaryColor = hw.bottom_color.blue();
                    }
                }
                while (opModeIsActive() && (dominantColor > (secondaryColor * multiplier))){
                    if (blue) {
                        dominantColor = hw.bottom_color.blue();
                        secondaryColor = hw.bottom_color.red();
                        multiplier = 1.25;
                    }
                    else {
                        dominantColor = hw.bottom_color.red();
                        secondaryColor = hw.bottom_color.blue();
                        multiplier = 1.25;
                    }
                }
                second = new Encoders(hw);
                middle = first.average(second);
                middle.applyOffset(desired, adjustedOffset);
                middle.set(drivetrain, hw);
                drivetrain.setPowerAll(speed);
                break;

            }

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
        double start = runtime.milliseconds();
        double timeout = 8000;
        Encoders e;
        e = new Encoders(hw);
        Encoders f = new Encoders(hw);
        int finaloffset = 150;
        double diff;
        boolean moving = true;
        double previousMs = runtime.milliseconds();
        while (opModeIsActive() && runtime.milliseconds() - start < timeout) {
            e = new Encoders(hw);
            double difference = e.totalDiff(middle);
            double avg = difference / 4;
            if (avg < 30) {
                break;
            }
        }

        drivetrain.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drivetrain.stop();
    }



    private void crypto(int ticks, double speed, double p, Direction direction) {
        int backLeftStart = hw.backLeft.getCurrentPosition();
        int backRightStart = hw.backRight.getCurrentPosition();
        int forwardLeftStart = hw.forwardLeft.getCurrentPosition();
        int forwardRightStart = hw.forwardRight.getCurrentPosition();
        int currentTicks = 0;
        while(opModeIsActive() && currentTicks < ticks) {
            double distance = hw.ultrasonic.distance();
            if (distance < 5) {

            }

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
        hw.jewelArm2.setPosition(0.85);
    }

    private void prestart(boolean red, boolean close) {
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            telemetry.addData("VuMark", "%s visible", vuMark);
            lastKnownVumark = vuMark;
        } else {
            telemetry.addData("VuMark", "not visible");
        }
        telemetry.addData("Red", red);
        telemetry.addData("Close", close);
        telemetry.update();
    }

    public void waitForStart(boolean red, boolean close) {
        while (!isStarted()) {
            synchronized (this) {
                try {
                    this.wait();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return;
                }
            }
            prestart(red, close);
        }
    }
}