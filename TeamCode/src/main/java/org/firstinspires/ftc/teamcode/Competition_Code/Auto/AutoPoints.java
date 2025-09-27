package org.firstinspires.ftc.teamcode.Competition_Code.Auto;

import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.Poses;


public enum AutoPoints {
    StartBlue(new Vector2d(-41,60), -Math.PI/2),
    AprilTagBlue(new Vector2d(-41,60), -Math.PI/2),
    LaunchBlue(new Vector2d(-41,60), -Math.PI/2),
    EndBlue(new Vector2d(-54,33), -Math.PI/2),

    PreIntakePPG(new Vector2d(-25,12), -Math.PI/2),
    PPGIntake1(new Vector2d(-36.5,12), -Math.PI/2),
    PPGIntake2(new Vector2d(-41.5,12), -Math.PI/2),
    PPGIntake3(new Vector2d(-48,12), -Math.PI/2),

    PreIntakePGP(new Vector2d(-25,-12), -Math.PI/2),
    PGPIntake1(new Vector2d(-36.5,-12), -Math.PI/2),
    PGPIntake2(new Vector2d(-41.5,-12), -Math.PI/2),
    PGPIntake3(new Vector2d(-48,-12), -Math.PI/2),

    PreIntakeGPP(new Vector2d(-25,-36), -Math.PI/2),
    GPPIntake1(new Vector2d(-36.5,-36), -Math.PI/2),
    GPPIntake2(new Vector2d(-41.5,-36), -Math.PI/2),
    GPPIntake3(new Vector2d(-48,-36), -Math.PI/2);




    AutoPoints(Vector2d vector, Double rotation) {
        runToExact = new SetDriveTarget(new Poses(vector,rotation));
    }

    public final SetDriveTarget runToExact;
}
