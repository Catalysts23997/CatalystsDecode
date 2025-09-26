package org.firstinspires.ftc.teamcode.Competition_Code.Auto;

import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.Poses;


public enum AutoPoints {
    StartBF(new Vector2d(-24,-63), 0.0),
    AprilTagBF(new Vector2d(0,0), 0.0),
    ShootBF(new Vector2d(0,0), 0.0),
    Ball1BF(new Vector2d(0,0), 0.0),
    Ball2BF(new Vector2d(0,0), 0.0),
    Ball3BF(new Vector2d(0,0), 0.0),
    EndBF(new Vector2d(43,-63), 0.0);



    AutoPoints(Vector2d vector, Double rotation) {
        runToExact = new SetDriveTarget(new Poses(vector,rotation));
    }

    public final SetDriveTarget runToExact;
}
