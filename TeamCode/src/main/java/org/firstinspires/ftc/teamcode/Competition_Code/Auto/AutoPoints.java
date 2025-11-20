package org.firstinspires.ftc.teamcode.Competition_Code.Auto;

import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.Poses;
import org.opencv.core.Mat;


public enum AutoPoints {
    StartBlue(new Vector2d(-39,63), 0.0),
    AprilTagBlue(new Vector2d(-15,45), Math.PI/8),
    LaunchBlue(new Vector2d(-15,25.0), 3*Math.PI/4),
    EndBlue(new Vector2d(-54,33), 0.0),

    PreIntakePPG(new Vector2d(-25,14), -Math.PI/2),
    PPGIntake1(new Vector2d(-36.5,14), -Math.PI/2),
    PPGIntake2(new Vector2d(-41.5,14), -Math.PI/2),
    PPGIntake3(new Vector2d(-55,14), -Math.PI/2),

    PreIntakePGP(new Vector2d(-25,-12), -Math.PI/2),
    PGPIntake1(new Vector2d(-36.5,-12), -Math.PI/2),
    PGPIntake2(new Vector2d(-41.5,-12), -Math.PI/2),
    PGPIntake3(new Vector2d(-63,-12), -Math.PI/2),
    PGPMidPoint(new Vector2d(-25,12), 9*Math.PI/8),


    PreIntakeGPP(new Vector2d(-25,-36), -Math.PI/2),
    GPPIntake1(new Vector2d(-36.5,-36), -Math.PI/2),
    GPPIntake2(new Vector2d(-41.5,-36), -Math.PI/2),
    GPPIntake3(new Vector2d(-63,-36), -Math.PI/2),
    GPPMidPoint(new Vector2d(-25,-12), 9*Math.PI/8),

    Test1(new Vector2d(-34,63), 0.0),
    Test2(new Vector2d(-39,68), Math.PI/2),
    Test3(new Vector2d(-44,63), 0.0),
    Test4(new Vector2d(-39,58), 0.0),

    StartRed(new Vector2d(39,63), 0.0),
    AprilTagRed(new Vector2d(0,50), -Math.PI/8),
    LaunchRed(new Vector2d(15.0,25.0), -3*Math.PI/4),
    EndRed(new Vector2d(54,33), 0.09),

    PreIntakePPGRed(new Vector2d(25,14), Math.PI/2),
    PPGIntake1Red(new Vector2d(36.5,14), Math.PI/2),
    PPGIntake2Red(new Vector2d(41.5,14), Math.PI/2),
    PPGIntake3Red(new Vector2d(55,14), Math.PI/2),

    PreIntakePGPRed(new Vector2d(25,-12), Math.PI/2),
    PGPIntake1Red(new Vector2d(36.5,-12), Math.PI/2),
    PGPIntake2Red(new Vector2d(41.5,-12), Math.PI/2),
    PGPIntake3Red(new Vector2d(63,-12), Math.PI/2),
    PGPMidPointRed(new Vector2d(25,12), -9*Math.PI/8),


    PreIntakeGPPRed(new Vector2d(25,-36), Math.PI/2),
    GPPIntake1Red(new Vector2d(36.5,-36), Math.PI/2),
    GPPIntake2Red(new Vector2d(41.5,-36), Math.PI/2),
    GPPIntake3Red(new Vector2d(63,-36), Math.PI/2),
    GPPMidPointRed(new Vector2d(25,-12), -9*Math.PI/8);


    AutoPoints(Vector2d vector, Double rotation) {
        runToExact = new SetDriveTarget(new Poses(vector,rotation));
    }

    public final SetDriveTarget runToExact;
}
