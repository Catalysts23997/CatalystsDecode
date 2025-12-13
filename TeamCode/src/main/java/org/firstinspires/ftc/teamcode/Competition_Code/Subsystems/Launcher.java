package org.firstinspires.ftc.teamcode.Competition_Code.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Launcher {

    /// The speed of the motor.
    private double speed = 0.0;

    /// The one motor that controls the launcher. There may be another one added in the
    /// future.
    DcMotor launcherMotor1;
    DcMotor launcherMotor2;

    /// Declare a new instance of the launcher system.
    ///
    /// # Only one instance should be active at a given time!
    public Launcher(HardwareMap hardwareMap) {
        launcherMotor1 = hardwareMap.get(DcMotor.class, "leftLauncher");
        // Make sure we know the default state of our motor
        launcherMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        launcherMotor1.setPower(0);
        launcherMotor2 = hardwareMap.get(DcMotor.class, "rightLauncher");
        // Make sure we know the default state of our motor
        launcherMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherMotor2.setPower(0);
    }

    /// Set the speed of the motor.
    /// This function includes a bounds check, so the speed can only be between 0.0 and 1.0
    public void setSpeed(double speed) {
        // Clamp the speed between 0.0 and 1.0
        this.speed = Math.max(0.0, Math.min(1.0, speed));
    }

    public double getSpeed() {
        return speed;
    }

    /// Stop the launcher!
    public void stop() {
        setSpeed(0.0);
    }

    /// This function will set the motor speed so that it can make the balls into the target from
    /// any location on the playing field.
    public void setSpeedFromLocation(double posX, double posY) {
        double motorSpeed = 0.0f;
        // TODO: Actually add the code to calculate the motor speed

        setSpeed(motorSpeed);
    }

    /// Tick the launcher
    public void update() {
        launcherMotor1.setPower(speed);
        launcherMotor2.setPower(speed);
    }

}
