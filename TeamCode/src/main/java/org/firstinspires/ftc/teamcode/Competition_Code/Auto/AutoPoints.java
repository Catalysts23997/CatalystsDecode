package org.firstinspires.ftc.teamcode.Competition_Code.Auto;

import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.Poses;


public enum AutoPoints {
    StartBlue(new Vector2d(39,-63), -Math.PI/2),
    AprilTagBlue(new Vector2d(20,-50), -Math.PI/4),
    LaunchBlue(new Vector2d(10,-20.0), -3*Math.PI/4),
    EndBlue(new Vector2d(54,-33), -Math.PI/2),

    PreIntakePPG(new Vector2d(25,-12), 0.0),
    PPGIntake1(new Vector2d(36.5,-12), 0.0),
    PPGIntake2(new Vector2d(41.5,-12), 0.0),
    PPGIntake3(new Vector2d(48,-12), 0.0),

    PreIntakePGP(new Vector2d(25,12), 0.0),
    PGPIntake1(new Vector2d(36.5,12), 0.0),
    PGPIntake2(new Vector2d(41.5,12), 0.0),
    PGPIntake3(new Vector2d(48,12), 0.0),
    PGPMidPoint(new Vector2d(25,-12), -3 * Math.PI / 8),


    PreIntakeGPP(new Vector2d(25,36), 0.0),
    GPPIntake1(new Vector2d(36.5,36), 0.0),
    GPPIntake2(new Vector2d(41.5,36), 0.0),
    GPPIntake3(new Vector2d(48,36), 0.0),
    GPPMidPoint(new Vector2d(25,12), -3 * Math.PI / 8),

    Test1(new Vector2d(0,-10), Math.PI/2),
    Test2(new Vector2d(0,10), -Math.PI/2),
    Test3(new Vector2d(45,-10), 0.0),
    Test4(new Vector2d(-45,10), Math.PI),

    StartRed(new Vector2d(-39,-63), Math.PI/2),
    AprilTagRed(new Vector2d(-20,-50), Math.PI/4),
    LaunchRed(new Vector2d(-10,-20.0), 3*Math.PI/4),
    EndRed(new Vector2d(-54,-33), Math.PI/2),

    PreIntakePPGRed(new Vector2d(-25,-12), 0.0),
    PPGIntake1Red(new Vector2d(-36.5,-12), 0.0),
    PPGIntake2Red(new Vector2d(-41.5,-12), 0.0),
    PPGIntake3Red(new Vector2d(-48,-12), 0.0),

    PreIntakePGPRed(new Vector2d(-25,12), 0.0),
    PGPIntake1Red(new Vector2d(-36.5,12), 0.0),
    PGPIntake2Red(new Vector2d(-41.5,12), 0.0),
    PGPIntake3Red(new Vector2d(-48,12), 0.0),
    PGPMidPointRed(new Vector2d(-25,-12), 3 * Math.PI / 8),


    PreIntakeGPPRed(new Vector2d(-25,36), 0.0),
    GPPIntake1Red(new Vector2d(-36.5,36), 0.0),
    GPPIntake2Red(new Vector2d(-41.5,36), 0.0),
    GPPIntake3Red(new Vector2d(-48,36), 0.0),
    GPPMidPointRed(new Vector2d(-25,12), 3 * Math.PI / 8);


    AutoPoints(Vector2d vector, Double rotation) {
        runToExact = new SetDriveTarget(new Poses(vector,rotation));
    }

    public final SetDriveTarget runToExact;
}
