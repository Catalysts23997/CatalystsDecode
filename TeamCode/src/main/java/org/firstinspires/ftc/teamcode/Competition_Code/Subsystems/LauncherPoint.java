package org.firstinspires.ftc.teamcode.Competition_Code.Subsystems;

import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.Competition_Code.AllianceColor;
import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.Poses;

public class LauncherPoint {

    // These variables store all the information from around the project
    // in one place.
    private static final LauncherPoint[] blueLauncherPoints = new LauncherPoint[] {
        new LauncherPoint(new Vector2d(-18, 17.5), 3 * Math.PI / 4, 3600, true, "Blue Main"),
        new LauncherPoint(new Vector2d(0, 53), 9.2*Math.PI/16, 3600, false, "Blue Alternative 1"),
        new LauncherPoint(new Vector2d(25, 55), 17*Math.PI/32, 4700, false, "Blue Alternative 2"),
        new LauncherPoint(new Vector2d(18, 17.5), 10.75*Math.PI/16, 5000, false, "Blue Alternative 3"), // yes, this is usually going to be the other team's main point :D
    };

    private static final LauncherPoint[] redLauncherPoints = flipPoints(blueLauncherPoints, "Blue", "Red");

    // nice helper function
    public static LauncherPoint[] flipPoints(LauncherPoint[] points, String nameReplaceSource, String nameReplaceValue) {
        LauncherPoint[] newPoints = points.clone();

        for (int i = 0; i < points.length; i++) {
            LauncherPoint point = points[i];

            newPoints[i] = new LauncherPoint(
                new Vector2d(-point.location.x, point.location.y),
                -point.rotation,
                point.launcherRPM,
                point.takePriority,
                point.displayName.replace(nameReplaceSource, nameReplaceValue)
            );
        }

        return newPoints;
    }

    public static LauncherPoint[] getLauncherPoints(AllianceColor color) {
        switch (color) {
            case Red: return redLauncherPoints;
            case Blue: return blueLauncherPoints;
        }

        // how did we get here????
        throw new IllegalStateException();
    }

    public static int getPriorityPoint(AllianceColor color) {
        LauncherPoint[] points = getLauncherPoints(color);

        for (int i = 0; i < points.length; i++) {
            if (points[i].takePriority) return i;
        }

        return 0; // default with the first one
    }

    private final Vector2d location;
    private final double rotation;
    private final double launcherRPM;
    private final boolean takePriority;
    public final String displayName;

    public LauncherPoint(
        Vector2d location,
        double rotation,
        double launcherRPM,
        boolean takePriority,
        String displayName
    ) {
        this.location = location;
        this.rotation = rotation;
        this.launcherRPM = launcherRPM;
        this.takePriority = takePriority;
        this.displayName = displayName;
    }

    public Vector2d getLocation() {
        return location;
    }

    public double getRotation() {
        return rotation;
    }

    public boolean takePriority() {
        return takePriority;
    }

    public double getLauncherRPM() {
        return launcherRPM;
    }

    public Poses getPose() {
        return new Poses(this.location, this.rotation);
    }

}
