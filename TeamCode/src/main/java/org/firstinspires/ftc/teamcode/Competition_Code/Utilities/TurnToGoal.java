package org.firstinspires.ftc.teamcode.Competition_Code.Utilities;

public class TurnToGoal {

    public double BlueTurn(double RobotX, double RobotY) {
        double DeltaX = -55 - RobotX;
        double DeltaY = 61 - RobotY;
        return Math.atan2(DeltaX, DeltaY);
    }
    public double RedTurn(double RobotX, double RobotY) {
        double DeltaX = 55 - RobotX;
        double DeltaY = 61 - RobotY;
        return Math.atan2(DeltaX, DeltaY);
    }
}
