package org.firstinspires.ftc.teamcode.Competition_Code.Auto;

import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.Poses;


public enum AutoPoints {
    StartBlue(new Vector2d(-39,63), 0.0),
    AprilTagBlue(new Vector2d(-15,45), Math.PI/8),

    PreLaunchBlue(new Vector2d(-40, 40), 3.1*Math.PI/4),
    LaunchFarBlue(new Vector2d(-14, -59), -3.52),
    LaunchBlue(new Vector2d(-18,17.5), 3.1*Math.PI/4),
    LaunchOffBlue(new Vector2d(-14,55), 9.5*Math.PI/16),
    EndBlue(new Vector2d(-35,0.0), 0.0, 1.5),
    OutOfTheWayBlue(new Vector2d(-29,60.0), 0.0),

    PreIntakePPG(new Vector2d(-25,13), -Math.PI / 2, 1.5),
    PPGIntake(new Vector2d(-55,13), -Math.PI / 2, 0.9, 1.5),

    GateMid(new Vector2d(-47,-10), 0.0),
    PreGate(new Vector2d(-47,5), 0.0),
    Gate(new Vector2d(-57,5), 0.0),


    PreIntakePGP(new Vector2d(-25,-10), -Math.PI / 2, 1.5),
    PGPIntake(new Vector2d(-64,-10), -Math.PI / 2, 0.9 , 1.5),
    PGPMidPoint(new Vector2d(-45,-10), 9 * Math.PI / 8, 1.5),

    PreIntakeGPP(new Vector2d(-25,-33), -Math.PI / 2, 1.5),
    GPPIntake(new Vector2d(-64,-33), -Math.PI / 2, 0.9, 1.5),
    GPPMidPoint(new Vector2d(-25,-33), 9*Math.PI / 8, 1.5),

    PreIntakeGPPFar(new Vector2d(-25,-36), -Math.PI / 2),
    GPPIntakeFar(new Vector2d(-64,-36), -Math.PI / 2, 0.9, 1.5),

    PGPIntakeFar(new Vector2d(-64,-62), -Math.PI / 2, 0.9, 4.0),
    PGPIntakeFarRed(new Vector2d(64,-62), Math.PI / 2, 0.9, 4.0),


    PreIntakeGPPFarRed(new Vector2d(25,-36), Math.PI / 2),
    GPPIntakeFarRed(new Vector2d(64,-36), Math.PI / 2, 0.9, 1.5),

    StartRed(new Vector2d(39,63), 0.0),
    AprilTagRed(new Vector2d(15,45), -Math.PI / 8),
    LaunchRed(new Vector2d(18.0,17.5), -3.0*Math.PI/4),
    LaunchOffRed(new Vector2d(14,55), -9.5*Math.PI/16),
    LaunchFarRed(new Vector2d(14, -59), 3.536),

    EndRed(new Vector2d(35,0.0), 0.0),
    OutOfTheWayRed(new Vector2d(29,60.0), 0.0),

    PreIntakePPGRed(new Vector2d(25,13), Math.PI / 2, 1.5),
    PPGIntakeRed(new Vector2d(55,13), Math.PI / 2, 0.9, 1.5),

    GateMidRed(new Vector2d(-47,-10), 0.0),
    PreGateRed(new Vector2d(47,5), 0.0),
    GateRed(new Vector2d(57,5), 0.0),

    PreIntakePGPRed(new Vector2d(25,-10), Math.PI / 2,1.5),
    PGPIntakeRed(new Vector2d(64,-10), Math.PI / 2, 0.9, 1.5),
    PGPMidPointRed(new Vector2d(45,-10), -9 * Math.PI / 8, 1.5),


    PreIntakeGPPRed(new Vector2d(25,-33), Math.PI / 2, 1.5),
    GPPIntakeRed(new Vector2d(64,-33), Math.PI / 2, 0.9, 1.5),
    GPPMidPointRed(new Vector2d(25,-33), -9 * Math.PI / 8, 1.5),

    StartFarRed(new Vector2d(15,-63), 0.0),
    StartFarBlue(new Vector2d(-15,-63), 0.0),
    MoveFarRed(new Vector2d(35,-63), 0.0),
    MoveFarBlue(new Vector2d(-35,-63), 0.0),

    EndgameBlue(new Vector2d(34,-38), 0.0),
    EndgameRed(new Vector2d(-34,-38), 0.0),

    Test1(new Vector2d(-34,63), 0.0),
    Test2(new Vector2d(-39,68), Math.PI / 2),
    Test3(new Vector2d(-44,63), 0.0),
    Test4(new Vector2d(-39,58), 0.0);

    public final double x,y, heading;
    public double driveSpeed = 1.0, maxTime = 11.0;

    AutoPoints(Vector2d vector, Double rotation) {
        this.x = vector.x;
        this.y = vector.y;
        this.heading = rotation;

        pose = new Poses(vector.x, vector.y, rotation);
    }
    AutoPoints(Vector2d vector, Double rotation, Double driveSpeed) {
        this.x = vector.x;
        this.y = vector.y;
        this.heading = rotation;
        this.driveSpeed = driveSpeed;

        pose = new Poses(vector.x, vector.y, rotation);
    }
    AutoPoints(Vector2d vector, Double rotation, Double driveSpeed, Double maxTime) {
        this.x = vector.x;
        this.y = vector.y;
        this.heading = rotation;
        this.driveSpeed = driveSpeed;
        this.maxTime = maxTime;

        pose = new Poses(vector.x, vector.y, rotation);
    }

    public SetDriveTarget runToExact(){
        return new SetDriveTarget(new Poses(this.x, this.y, this.heading), this.driveSpeed, this.maxTime);
    }

    public SetDriveTarget runToFast(){
        return new SetDriveTarget(new Poses(this.x, this.y, this.heading), this.driveSpeed, this.maxTime, 10.0, 15.0);
    }

    public final Poses pose;
}
