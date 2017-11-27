package org.firstinspires.ftc.teamcode.Shared;

import com.qualcomm.robotcore.hardware.DcMotor;

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

    final double rotSpeed = 0.5;
    final double speedThreshold = 0.05;
    final double turnThreshold = 2;

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

    public void turn(boolean clockwise, double speed, double seconds) throws InterruptedException{
        Rotate(clockwise, speed);
        wait((long)(seconds * 1000));
        stop();
    }

    double normalize(double angle) {
        if (angle < 0) {
            angle += 360;
        }
        if (angle >= 360) {
            angle -= 360;
        }
        return angle;
    }

    public void GyroTurn(double speed, double angle) {
        double x1 = cos(angle);
        double y1 = sin(angle);

        double heading = normalize(getHeading());

        double x2 = cos(heading);
        double y2 = sin(heading);

        double c = (x1 * y2) - (y1 * x2);
        if (c >= 0) {
            // CCW
            MoveAngle(0, 0, speed);

        }
        else if (c < 0) {
            // CW
            MoveAngle(0, 0, -speed);
        }

        double lower = normalize(angle - turnThreshold);
        double upper = normalize(angle + turnThreshold);
        while (true) {
            heading = normalize(getHeading());
            if (heading > lower && heading < upper) {
                stop();
                break;
            }
        }
    }

    public void OneEighty() {

    }

    public float getHeading() {
        return rw.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

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

    public void stop() {
        rw.forwardRight.setPower(0);
        rw.forwardLeft.setPower(0);
        rw.backRight.setPower(0);
        rw.backLeft.setPower(0);
    }

    public void setMotorZeroPower(DcMotor.ZeroPowerBehavior zeroPower) {
        rw.forwardRight.setZeroPowerBehavior(zeroPower);
        rw.forwardLeft.setZeroPowerBehavior(zeroPower);
        rw.backRight.setZeroPowerBehavior(zeroPower);
        rw.backLeft.setZeroPowerBehavior(zeroPower);
    }

    public void setSpeedMultiplier(double speedMultiplier) {
        this.speedMultiplier = speedMultiplier;
    }

}
