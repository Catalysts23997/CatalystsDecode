package org.firstinspires.ftc.teamcode.Competition_Code.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Launcher {

    public double leftRpm;
    public double rightRpm;

    /// The one motor that controls the launcher. There may be another one added in the
    /// future.
    DcMotor leftLauncher;
    DcMotor rightLauncher;

    public double leftPower = 0;
    public double rightPower = 0;


    final double ticksPerRev = 537.7;
    ElapsedTime timer = new ElapsedTime();


    /// Declare a new instance of the launcher system.
    ///
    /// # Only one instance should be active at a given time!
    public Launcher(HardwareMap hardwareMap) {
        leftLauncher = hardwareMap.get(DcMotor.class, "leftLauncher");
        // Make sure we know the default state of our motor
        leftLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
        leftLauncher.setPower(0);
        rightLauncher = hardwareMap.get(DcMotor.class, "rightLauncher");
        // Make sure we know the default state of our motor
        rightLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLauncher.setPower(0);
        timer.reset();

        leftLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }


    double power;

    public void setSpeed(double power) {
        // Clamp the proportion between 0.0 and 1.0
        this.power = power;
    }

    public double getLeftRpm() {
        return leftRpm;
    }
    public double getRightRpm() {
        return rightRpm;
    }

    /// Stop the launcher!
    public void stop() {
        power = 0;
    }

    double leftLastPos = 0;
    double rightLastPos = 0;


    public void update() {
        double dt = timer.seconds();
        if (dt < 0.05) return;  // updatePID every 50ms
        timer.reset();

        double leftPos = leftLauncher.getCurrentPosition();
        double rightPos = rightLauncher.getCurrentPosition();

        leftRpm = 60/dt * (leftPos-leftLastPos)/ticksPerRev;
        rightRpm = 60/dt *(rightPos -rightLastPos)/ticksPerRev;

        leftLastPos = leftPos;
        rightLastPos = rightPos;

        leftLauncher.setPower(power);
        rightLauncher.setPower(power);
    }

}
