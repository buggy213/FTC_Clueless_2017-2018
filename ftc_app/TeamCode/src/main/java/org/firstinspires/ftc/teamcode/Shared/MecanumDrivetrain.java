package org.firstinspires.ftc.teamcode.Shared;

/**
 * Created by Joshua on 9/9/2017.
 */

public interface MecanumDrivetrain {
    public void MoveCardinal(Direction direction, float speed);

    public void MoveAngle(float speed, float angle);
}
