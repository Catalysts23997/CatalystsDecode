package org.firstinspires.ftc.teamcode.Competition_Code.Auto;

import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.Poses;


public enum AutoPoints {
    //Main Blocks
    FastStartBlue(new Vector2d(-52,59), 11*Math.PI/16),
    FastStartRed(new Vector2d(50,59), -11.2*Math.PI/16),

    AprilTagBlue(new Vector2d(-15,45), Math.PI/8),
    LaunchBlue(new Vector2d(-18,17.5), 2.95*Math.PI/4),
    EjectBlue(new Vector2d(-18,17.5),  Math.PI),
    AprilTagRed(new Vector2d(15,45), -Math.PI / 8),
    LaunchRed(new Vector2d(18.0,17.5), -2.9*Math.PI/4),
    EjectRed(new Vector2d(18,17.5),  Math.PI),

    PrePPGBlue(new Vector2d(-25,13), -Math.PI / 2),
    PPGBlue(new Vector2d(-56,13), -Math.PI / 2, 0.6, 1.5),
    PrePPGRed(new Vector2d(25,12.5), Math.PI / 2),
    PPGRed(new Vector2d(57,12.5), Math.PI / 2, 0.6, 1.5),


    PrePGPBlue(new Vector2d(-25,-11.3), -Math.PI / 2),
    PGPBlue(new Vector2d(-63,-11.3), -Math.PI / 2, 0.6 , 1.5),
    PGPMidBlue(new Vector2d(-45,-11.3), 9 * Math.PI / 8),
    PGPBackBlue(new Vector2d(-25,-11.3), 9 * Math.PI / 8),
    PrePGPRed(new Vector2d(25,-11.5), Math.PI / 2),
    PGPRed(new Vector2d(65,-11.5), Math.PI / 2, 0.6, 1.5),
    PGPMidRed(new Vector2d(45,-11.5), -9 * Math.PI / 8),
    PGPBackRed(new Vector2d(25,-11.5), -9 * Math.PI / 8),

    PreGPPBlue(new Vector2d(-23,-33.5), -Math.PI / 2),
    GPPBlue(new Vector2d(-63,-33.5), -Math.PI / 2, 0.6, 1.5),
    GPPBackBlue(new Vector2d(-23,-33.5), 9*Math.PI / 8),
    PreGPPRed(new Vector2d(23,-34), Math.PI / 2),
    GPPRed(new Vector2d(65,-34), Math.PI / 2, 0.6, 1.5),
    GPPBackRed(new Vector2d(25,-34), -9 * Math.PI / 8),

    EndBlue(new Vector2d(-45,0.0), 0.0),
    EndRed(new Vector2d(45,0.0), 0.0),

    //Gate Empty
    GateMidBlue(new Vector2d(-47,-10), 0.0),
    PreGateBlue(new Vector2d(-47,2.5), 0.0),
    GateBlue(new Vector2d(-57,2.5), 0.0, 1.0, 1.5),
    GateMidRed(new Vector2d(47,-12), 0.0),
    PreGateRed(new Vector2d(47,4), 0.0),
    GateRed(new Vector2d(58,4), 0.0, 1.0, 1.5),

    //Gate Intake
    PreGateIntakeBlue(new Vector2d(-58,-8.5), -3*Math.PI/8, 1.5, 1.5),
    GateIntakeBlue1(new Vector2d(-61,-20), -2.9*Math.PI/8, 1.5, 1.5),
    GateIntakeBlue2(new Vector2d(-61,-15), -2.9*Math.PI/8, 1.5, 1.5),
    GateIntakePrepRed(new Vector2d(60,-8), 3*Math.PI/8, 1.5, 1.5),
    GateIntakeRed1(new Vector2d(61,-20), 2.9*Math.PI/8, 1.5, 1.5),
    GateIntakeRed2(new Vector2d(61,-15), 2.9*Math.PI/8, 1.5, 1.5),

    //Far
    StartFarBlue(new Vector2d(-15,-63), Math.PI),
    LaunchFarBlue(new Vector2d(-14, -59), -3.52),
    StartFarRed(new Vector2d(15,-63), Math.PI),
    LaunchFarRed(new Vector2d(14, -59), 3.536),

    PreGPPFarBlue(new Vector2d(-25,-38), -Math.PI / 2),
    GPPFarBlue(new Vector2d(-60,-38), -Math.PI / 2, 0.8, 1.5),
    PreGPPFarRed(new Vector2d(25,-38), Math.PI / 2),
    GPPFarRed(new Vector2d(60,-38), Math.PI / 2, 0.8, 1.5),

    PreCornerBlue(new Vector2d(-59,-45), -5*Math.PI / 8, 1.0, 1.5),
    CornerBlue(new Vector2d(-63,-59), -4*Math.PI / 8, 1.5, 1.8),
    PreCornerRed(new Vector2d(59,-45), 5*Math.PI / 8, 1.0, 1.5),
    CornerRed(new Vector2d(63,-59), 4*Math.PI / 8, 1.5, 1.8),

    EndFarBlue(new Vector2d(-35,-60), Math.PI),
    EndFarRed(new Vector2d(35,-60), Math.PI),

    //Other
    OldStartBlue(new Vector2d(-39,63), 0.0),
    OldStartRed(new Vector2d(39,63), 0.0),

    BlueLaunchFar2(new Vector2d(-12, -55), -3.58),

    MovingLaunchBlue(new Vector2d(-40, 40), 3.1*Math.PI/4),
    LaunchBlueSlow(new Vector2d(-18,17.5), 3*Math.PI/4, 0.4),
    MovingLaunchRed(new Vector2d(40, 40), -3.1*Math.PI/4),
    LaunchRedSlow(new Vector2d(18,17.5), -3*Math.PI/4, 0.4),

    LaunchOffBlue(new Vector2d(-14,55), 9.5*Math.PI/16),
    OutOfTheWayBlue(new Vector2d(-29,60.0), 0.0),
    LaunchOffRed(new Vector2d(14,55), -9.5*Math.PI/16),
    OutOfTheWayRed(new Vector2d(29,60.0), 0.0),

    EndgameBlue(new Vector2d(34,-38), 0.0),
    EndgameRed(new Vector2d(-34,-38), 0.0),

    //testing
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
