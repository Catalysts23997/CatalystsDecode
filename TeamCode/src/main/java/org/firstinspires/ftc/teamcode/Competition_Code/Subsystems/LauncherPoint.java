package org.firstinspires.ftc.teamcode.Competition_Code.Subsystems;

import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.Poses;

public class LauncherPoint {

    // These variables store all the information from around the project
    // in one place.
    public static final LauncherPoint[] blueLauncherPoints = new LauncherPoint[] {
        new LauncherPoint(new Vector2d(-18, 17.5), 3 * Math.PI / 4, 3500, true, "Blue Main"),
        new LauncherPoint(new Vector2d(-12, 55), 9 * Math.PI / 16, 3500, true, "Blue Alternative 1"),
        new LauncherPoint(new Vector2d(-57, 16.3), 0, 3500, true, "Blue Alternative 2"),
        new LauncherPoint(new Vector2d(-29.4, 63.2), -(Math.PI / 3), 3500, true, "Blue Alternative 3"),
        new LauncherPoint(new Vector2d(0, -21.3), -(Math.PI / 2), 3500, true, "Blue Alternative 4"),
        new LauncherPoint(new Vector2d(38, 38), -(Math.PI / 2.3), 3500, true, "Blue Alternative 5"),
        new LauncherPoint(new Vector2d(18, 17.5), -(3 * Math.PI / 4), 3500, true, "Blue Alternative 6"), // yes, this is usually going to be the other team's main point :D
    };

    public static final LauncherPoint[] redLauncherPoints = flipPoints(blueLauncherPoints, "Blue", "Red");

    // nice helper function
    public static LauncherPoint[] flipPoints(LauncherPoint[] points, String nameReplaceSource, String nameReplaceValue) {
        LauncherPoint[] newPoints = points.clone();

        for (int i = 0; i < points.length; i++) {
            LauncherPoint point = points[i];

            newPoints[i] = new LauncherPoint(
                new Vector2d(-point.location.x, -point.location.y),
                -point.rotation,
                point.launcherRPM,
                point.takePriority,
                point.displayName.replace(nameReplaceSource, nameReplaceValue)
            );
        }

        return newPoints;
    }

    private final Vector2d location;
    private final double rotation;
    private final int launcherRPM;
    private final boolean takePriority;
    public final String displayName;

    public LauncherPoint(
        Vector2d location,
        double rotation,
        int launcherRPM,
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

    public int getLauncherRPM() {
        return launcherRPM;
    }

    public Poses getPose() {
        return new Poses(this.location, this.rotation);
    }

    public static int getPriorityPoint(LauncherPoint[] points) {
        for (int i = 0; i < points.length; i++) {
            if (points[i].takePriority) return i;
        }

        return 0; // default with the first one
    }
}
