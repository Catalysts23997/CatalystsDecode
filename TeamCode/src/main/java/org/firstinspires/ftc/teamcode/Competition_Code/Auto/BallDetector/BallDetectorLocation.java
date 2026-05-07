package org.firstinspires.ftc.teamcode.Competition_Code.Auto.BallDetector;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Competition_Code.Auto.SetDriveTarget;
import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.Poses;

public class BallDetectorLocation {

    public final Poses pose;
    public double driveSpeed;
    public long maxTime;

    public BallDetectorLocation(Poses pose) {
        this(pose, 1.0);
    }

    public BallDetectorLocation(Poses pose, double driveSpeed) {
        this(pose, driveSpeed, 11 * 1000);
    }

    public BallDetectorLocation(Poses pose, double driveSpeed, long maxTime) {
        this.pose = pose;
        this.driveSpeed = driveSpeed;
        this.maxTime = maxTime;
    }

    public Action Drive() {
        Poses pose = this.pose;
        double driveSpeed = this.driveSpeed;
        long maxTime = this.maxTime;

        return new Action() {
            private final ElapsedTime timer = new ElapsedTime();
            private final SetDriveTarget driver = new SetDriveTarget(pose, driveSpeed, 100.0); // some large number for the max time. It is not used anyway!

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                // check the timer
                if (timer.milliseconds() >= maxTime) {
                    return false;
                }

                // move the robot
                driver.run(telemetryPacket);
                return true;
            }
        };
    }

    @Override
    public boolean equals(Object obj) {
        if (obj instanceof BallDetectorLocation) {
            BallDetectorLocation location = (BallDetectorLocation) obj;

            return location.pose.equals(this.pose) && location.maxTime == this.maxTime && location.driveSpeed == this.driveSpeed;
        } else {
            // can never be equal
            return false;
        }
    }

    @NonNull
    @Override
    public String toString() {
        return this.pose.getX() + ", " + this.pose.getY() + " H " + this.pose.getHeading() + " S " + this.driveSpeed + " T " + this.maxTime;
    }
}
