package org.firstinspires.ftc.teamcode.Shared;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.MotorConfiguration;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

/**
 * Created by Joshua on 9/9/2017.
 */

public class FourWheelMecanumDrivetrain implements MecanumDrivetrain {
    RobotHardware rw = RobotHardware.GetSingleton();

    Orientation angles;

    double speedMultiplier = 0.75;

    // Constants used to adjust various parameters / characteristics of the drivetrain
    final double rotSpeed = 0.5;
    final double speedThreshold = 0.05;
    final double turnThreshold = 2;

    // region auto
    // Primary movement method for auto
    public void AutoMove(double speed, double angle, int counts) {
        int initialForward = rw.forwardLeft.getCurrentPosition();
        int initialBackward = rw.backLeft.getCurrentPosition();

        MoveAngle(speed, angle, 0);

        while (true) {
            int differenceForward = Math.abs(rw.forwardLeft.getCurrentPosition() - initialForward);
            int differenceBackward = Math.abs(rw.backLeft.getCurrentPosition() - initialBackward);

            if ((differenceBackward + differenceForward) / 2 > counts) {
                stop();
                break;
            }
        }
    }

    public void AutoMove(double speed, double angle, double time) throws InterruptedException{
        MoveAngle(speed, angle, 0);
        Thread.sleep((long)(time * 1000));
        stop();
    }

    public void AutoMove(Direction direction, double speed, double time) throws InterruptedException{
        MoveCardinal(direction, (float)speed);
        Thread.sleep((long)(time * 1000));
        stop();
    }

    // "Dumb" turn, based on time
    public void turn(boolean clockwise, double speed, double seconds) throws InterruptedException{
        Rotate(clockwise, speed);
        Thread.sleep((long)(seconds * 1000));
        stop();
    }

    // Gyroscope Sensor based turn, untested
    public void GyroTurn(double speed, double angle) {
        // Angle is counterclockwise (sorry)
        double normalizedHeading = normalize(getHeading());
        double normalizedAngle = normalize(angle);
        double angleDiff = normalizedHeading - normalizedAngle;

        angleDiff = (angleDiff / 180) * Math.PI;
        double c = sin(angleDiff);
        if (c >= 0) {
            // CW
            MoveAngle(0, 0, speed);

        }
        else if (c < 0) {
            // CCW
            MoveAngle(0, 0, -speed);
        }

        while (true) {
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
                stop();
                break;
            }
        }
    }

    // Pulls a one-eighty using the gyro, doesn't need to be precise
    public void OneEighty() {

    }

    public float getHeading() {
        return rw.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    //endregion

    @Override
    public void MoveCardinal(Direction direction, float speed) {
        switch (direction) {
            case FORWARD:
                rw.forwardRight.setPower(speed);
                rw.forwardLeft.setPower(speed);
                rw.backRight.setPower(speed);
                rw.backLeft.setPower(speed);
                break;
            case BACKWARD:
                rw.forwardRight.setPower(-speed);
                rw.forwardLeft.setPower(-speed);
                rw.backRight.setPower(-speed);
                rw.backLeft.setPower(-speed);
                break;
            case LEFT:
                rw.forwardRight.setPower(speed);
                rw.forwardLeft.setPower(-speed);
                rw.backRight.setPower(-speed);
                rw.backLeft.setPower(speed);
                break;
            case RIGHT:
                rw.forwardRight.setPower(-speed);
                rw.forwardLeft.setPower(speed);
                rw.backRight.setPower(speed);
                rw.backLeft.setPower(-speed);
                break;

        }
    }

    // Turns robot
    public void Rotate(boolean clockwise, double speed) {
        if (clockwise) {
            setPower(rw.forwardRight, speed);
            setPower(rw.forwardLeft, -speed);
            setPower(rw.backRight, speed);
            setPower(rw.backLeft, -speed);
        }
        else {
            setPower(rw.forwardRight, -speed);
            setPower(rw.forwardLeft, speed);
            setPower(rw.backRight, -speed);
            setPower(rw.backLeft, speed);
        }
    }

    // Moves robot relative to the field, rather than itself. Fails miserably when having to move diagonally in a consistent manner
    public void FieldOrientedDrive(double speed, double forward, double strafe, double turn) {
        float heading = getHeading();

        double gyro_radians = heading * Math.PI/180;
        double temp = forward * cos(gyro_radians) +
                strafe * sin(gyro_radians);
        strafe = -forward * sin(gyro_radians) +
                strafe * cos(gyro_radians);
        forward = temp;

        double angle = Math.atan2(strafe, -forward);

        MoveAngle(speed, angle, turn);
    }

    // Primary movement methods

    /**
     *
     * @param speed The speed of the robot from -1 to 1
     * @param angle Angle (in radians) that the robot should go
     * @param turn Turning velocity
     */
    @Override
    public void MoveAngle(double speed, double angle, double turn) {
        double vRot = turn;

        double desiredAngle = (angle) + Math.PI / 4;
        if (desiredAngle < 0) {
            desiredAngle = desiredAngle + 2 * Math.PI;
        }
        if (desiredAngle >= 2 * Math.PI) {
            desiredAngle = desiredAngle % (2 * Math.PI);
        }

        double intermediateSin = sin(desiredAngle);
        double intermediateCos = cos(desiredAngle);

        double leftForward = speed * (intermediateSin) + vRot;
        double leftBackward = speed * (intermediateCos) + vRot;
        double rightForward = speed * (intermediateCos) - vRot;
        double rightBackward = speed * (intermediateSin) - vRot;

        if (Math.abs(rightBackward) < speedThreshold) {
            rightBackward = 0;
        }
        if (Math.abs(rightForward) < speedThreshold) {
            rightForward = 0;
        }
        if (Math.abs(leftBackward) < speedThreshold) {
            leftBackward = 0;
        }
        if (Math.abs(leftForward) < speedThreshold) {
            leftForward = 0;
        }
        setPower(rw.forwardRight, rightForward);
        setPower(rw.forwardLeft, leftForward);
        setPower(rw.backRight, rightBackward);
        setPower(rw.backLeft, leftBackward);
    }

    public void setPower(DcMotor motor, double speed) {
        motor.setPower((speed * speedMultiplier));
    }

    // Ceases all movement
    public void stop() {
        rw.forwardRight.setPower(0);
        rw.forwardLeft.setPower(0);
        rw.backRight.setPower(0);
        rw.backLeft.setPower(0);
    }

    // Blanket sets all zero power behaviours for the entire drivetrain
    public void setMotorZeroPower(DcMotor.ZeroPowerBehavior zeroPower) {
        rw.forwardRight.setZeroPowerBehavior(zeroPower);
        rw.forwardLeft.setZeroPowerBehavior(zeroPower);
        rw.backRight.setZeroPowerBehavior(zeroPower);
        rw.backLeft.setZeroPowerBehavior(zeroPower);
    }
    
    public void setMotorMode(DcMotor.RunMode runMode) {
        rw.forwardRight.setMode(runMode);
        rw.forwardLeft.setMode(runMode);
        rw.backRight.setMode(runMode);
        rw.backLeft.setMode(runMode);
    }

    // Sets the "overall" speed of the drivetrain
    public void setSpeedMultiplier(double speedMultiplier) {
        this.speedMultiplier = speedMultiplier;
    }

    double normalize(double angle) {
        angle = (360 + angle % 360) % 360;
        return angle;
    }

}
