package org.firstinspires.ftc.teamcode.Competition_Code.Auto;

public enum DetectionStage {
    Waiting,
    DrivingFirst,
    DrivingSecond,
    Done;

    public DetectionStage nextStage() {
        switch (this) {
            case Waiting: return DrivingFirst;
            case DrivingFirst: return DrivingSecond;

            // both lead to Done
            case DrivingSecond:
            case Done:
            default:
                return Done;
        }
    }
}
