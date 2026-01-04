package org.firstinspires.ftc.teamcode.Competition_Code.Subsystems;

import static org.firstinspires.ftc.teamcode.Competition_Code.Utilities.NormalizeKt.normalize;
import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Competition_Code.PinpointLocalizer.Localizer;
import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.Angles;
import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.Poses;

public class DrivetrainOverride {

    private boolean shouldOverridingInput = false;
    private Poses target;

    double k = 1.0;

    public boolean shouldOverrideInput() {
        return shouldOverridingInput;
    }

    public void beginOverriding(Poses target) {
        this.target = target;
        this.shouldOverridingInput = true;
    }

    public void stopOverriding() {
        this.target = null;
        this.shouldOverridingInput = false;
    }

    /// This code does updatePID the drivetrain, and should only be called
    /// when the override is active
    public void update(Drivetrain drive) {
        // prevent this from running if we don't have a target!
        if (null == target) return;

        // Calculate some math
        Poses current = Localizer.pose;

        double latError = target.getY() - current.getY();
        double axialError = target.getX() - current.getX();
        double headingError = Angles.INSTANCE.wrap(
            target.getHeading() - current.getHeading()
        );

        double lateral = drive.getYpid().calculate(latError);
        double axial = drive.getXpid().calculate(axialError);
        double turn = drive.getRpid().calculate(headingError);

        double heading = -current.getHeading();
        double rotX = -axial * Math.cos(heading) - lateral * Math.sin(heading);
        double rotY = -axial * Math.sin(heading) + lateral * Math.cos(heading);

        double powerLF = k*(rotY - rotX + turn);
        double powerLB = k*(rotY + rotX + turn);
        double powerRF = k*(rotY + rotX - turn);
        double powerRB= k*(rotY - rotX - turn);

        double[] powers = {powerLF, powerLB, powerRF, powerRB};

        powers = normalize(powers, 1.0);

        drive.getLeftFront().setPower(powers[0]);
        drive.getLeftBack().setPower(powers[1]);
        drive.getRightFront().setPower(powers[2]);
        drive.getRightBack().setPower(powers[3]);

        //removed so that it will continue to hold the position if bumped
//        if (abs(axialError) <= 1.0 &&
//            abs(latError) <= 1.0 &&
//            abs(headingError) <= Math.toRadians(5.0)) {
//            stopOverriding();
//        }
    }

    /// This function just checks to see if the user would like
    /// to have input again!
    ///
    /// This function will return `true` if the override was
    /// disabled because the user requested control or `false`
    /// if not.
    public boolean safetyMeasures(Gamepad gamepad) {
        if (gamepad.x ||
            abs(gamepad.right_stick_x) >= 0.1 || abs(gamepad.right_stick_y) >= 0.1 ||
                abs(gamepad.left_stick_x) >= 0.1 ||abs(gamepad.left_stick_y) >= 0.1) {
            stopOverriding();

            return true;
        }

        if(gamepad.y || gamepad.b){
            k = 1.5;
        } else k = 1;

        return false;
    }

}
