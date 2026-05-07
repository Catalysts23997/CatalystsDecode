package org.firstinspires.ftc.teamcode.Competition_Code.Auto.BallDetector;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.Poses;

public class BallDetectorGlobals {

    public static final boolean IS_DEBUG = true;

    // constants
    private static final BallDetectorLocation CONST_LOCATION_WAIT = new BallDetectorLocation(
        new Poses(-60,-59, -0.3)
    );
    private static final BallDetectorLocation CONST_LOCATION_PICKUP_FIRST = new BallDetectorLocation(
        new Poses(-65.0, -43.0, -0.95),
        1.0,
        700
    );
    private static final BallDetectorLocation CONST_LOCATION_PICKUP_SECOND = new BallDetectorLocation(
        new Poses(-65.0, -17.5, -0.83),
        0.9,
        1500
    );

    // variables
    public static BallDetectorLocation LOCATION_WAIT = CONST_LOCATION_WAIT;
    public static BallDetectorLocation LOCATION_PICKUP_FIRST = CONST_LOCATION_PICKUP_FIRST;
    public static BallDetectorLocation LOCATION_PICKUP_SECOND = CONST_LOCATION_PICKUP_SECOND;

    public static void resetGlobalsForComp(Telemetry telemetry) {
        if (!IS_DEBUG) {
            if (LOCATION_WAIT != CONST_LOCATION_WAIT) { LOCATION_WAIT = CONST_LOCATION_WAIT; telemetry.addLine("Restoring wait position..."); }
            if (LOCATION_PICKUP_FIRST != CONST_LOCATION_PICKUP_FIRST) { LOCATION_PICKUP_FIRST = CONST_LOCATION_PICKUP_FIRST; telemetry.addLine("Restoring pickup first position..."); }
            if (LOCATION_PICKUP_SECOND != CONST_LOCATION_PICKUP_SECOND) { LOCATION_PICKUP_SECOND = CONST_LOCATION_PICKUP_SECOND; telemetry.addLine("Restoring pickup second position..."); }
        }

        // we don't do anything in debug mode!
    }

}
