package org.firstinspires.ftc.teamcode.Competition_Code.Auto;

import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.Poses;


public enum AutoPoints {
    StartBlue(new Vector2d(-39,63), 0.0), // todo update new start position (robto has to touch tape)
    AprilTagBlue(new Vector2d(-15,45), Math.PI/8),
    LaunchBlue(new Vector2d(-18,19.0), 3*Math.PI/4),
    EndBlue(new Vector2d(-25,0.0), 0.0),
    OutOfTheWayBlue(new Vector2d(-50,30.0), 0.0),

    PreIntakePPG(new Vector2d(-25,14), -Math.PI / 2, .5),
    PPGIntake(new Vector2d(-56,14), -Math.PI / 2, 6.0),

    PreIntakePGP(new Vector2d(-25,-11), -Math.PI / 2,.5),
    PGPIntake(new Vector2d(-64,-11), -Math.PI / 2, 1.0 , 6.0),
    PGPMidPoint(new Vector2d(-25,-11), 9 * Math.PI / 8),

    PreIntakeGPP(new Vector2d(-25,-33), -Math.PI / 2,.5),
    GPPIntake(new Vector2d(-64,-33), -Math.PI / 2, 1.0, 6.0),
    GPPMidPoint(new Vector2d(-25,-33), 9*Math.PI / 8),

    StartRed(new Vector2d(39,63), 0.0),
    AprilTagRed(new Vector2d(15,45), -Math.PI / 8),
    LaunchRed(new Vector2d(18.0,19.0), -3 * Math.PI / 4),
    EndRed(new Vector2d(25,0.0), 0.0),
    OutOfTheWayRed(new Vector2d(50,30.0), 0.0),

    PreIntakePPGRed(new Vector2d(25,14), Math.PI / 2,.5),
    PPGIntakeRed(new Vector2d(56,14), Math.PI / 2, 1.0, 6.0),

    PreIntakePGPRed(new Vector2d(25,-11), Math.PI / 2,.5),
    PGPIntakeRed(new Vector2d(64,-11), Math.PI / 2, 1.0, 6.0),
    PGPMidPointRed(new Vector2d(25,-11), -9 * Math.PI / 8),


    PreIntakeGPPRed(new Vector2d(25,-33), Math.PI / 2,.5),
    GPPIntakeRed(new Vector2d(64,-33), Math.PI / 2, 1.0, 6.0),
    GPPMidPointRed(new Vector2d(25,-33), -9 * Math.PI / 8),

    StartFarRed(new Vector2d(15,-63), 0.0),
    StartFarBlue(new Vector2d(-15,-63), 0.0),
    MoveFarRed(new Vector2d(35,-63), 0.0),
    MoveFarBlue(new Vector2d(-35,-63), 0.0),

    Test1(new Vector2d(-34,63), 0.0),
    Test2(new Vector2d(-39,68), Math.PI / 2),
    Test3(new Vector2d(-44,63), 0.0),
    Test4(new Vector2d(-39,58), 0.0);

    AutoPoints(Vector2d vector, Double rotation) {
        runToExact = new SetDriveTarget(new Poses(vector,rotation));
        pose = new Poses(vector.x, vector.y, rotation);
    }
    AutoPoints(Vector2d vector, Double rotation, Double driveSpeed) {
        runToExact = new SetDriveTarget(new Poses(vector,rotation), driveSpeed);
        pose = new Poses(vector.x, vector.y, rotation);
    }
    AutoPoints(Vector2d vector, Double rotation, Double driveSpeed, Double maxTime) {
        runToExact = new SetDriveTarget(new Poses(vector, rotation), driveSpeed, maxTime);
        pose = new Poses(vector.x, vector.y, rotation);
    }


    public final SetDriveTarget runToExact;
    public final Poses pose;
}
