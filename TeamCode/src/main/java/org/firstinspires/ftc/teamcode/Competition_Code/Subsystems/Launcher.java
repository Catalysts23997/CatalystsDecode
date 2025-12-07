package org.firstinspires.ftc.teamcode.Competition_Code.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.PIDController;
import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.PIDParams;

public class Launcher {

    double goalRpm;
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

    PIDParams leftPidParams = new PIDParams(0,0,0,0);
    PIDController leftController = new PIDController(leftPidParams);

    PIDParams rightPidParams = new PIDParams(0,0,0,0);
    PIDController rightController = new PIDController(rightPidParams);

    public void setLeftPidParams(PIDParams params) {
        leftPidParams = params;
        leftController.setPID(leftPidParams);
    }
    public void setRightPidParams(PIDParams params) {
        rightPidParams = params;
        rightController.setPID(rightPidParams);
    }
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

    /// Set the proportion of the motor.
    /// This function includes a bounds check, so the proportion can only be between 0.0 and 1.0
    public void setRPM(double rpm) {
        // Clamp the proportion between 0.0 and 1.0
        this.goalRpm = rpm;
    }

    double power;

    public void setPower(double power) {
        // Clamp the proportion between 0.0 and 1.0
        this.power = power;
    }

    public double getGoalRpm() {
        return goalRpm;
    }
    public double getLeftRpm() {
        return leftRpm;
    }
    public double getRightRpm() {
        return rightRpm;
    }

    /// Stop the launcher!
    public void stop() {
        setRPM(0.0);
    }

    /// This function will set the motor proportion so that it can make the balls into the target from
    /// any location on the playing field.
    public void setSpeedFromLocation(double posX, double posY) {
        double motorSpeed = 0.0f;
        // TODO: Actually add the code to calculate the motor proportion

        setRPM(motorSpeed);
    }

    double leftLastPos = 0;
    double rightLastPos = 0;

    /// Tick the launcher
    public void updatePID() {
        double dt = timer.seconds();
        if (dt < 0.05) return;  // updatePID every 50ms
        timer.reset();

        double leftPos = leftLauncher.getCurrentPosition();
        double rightPos = rightLauncher.getCurrentPosition();

        leftRpm = 60/dt * (leftPos-leftLastPos)/ticksPerRev;
        rightRpm = 60/dt *(rightPos -rightLastPos)/ticksPerRev;

        leftLastPos = leftPos;
        rightLastPos = rightPos;


        leftController.setPID(leftPidParams);
        rightController.setPID(rightPidParams);

        leftPower = leftController.calculate(goalRpm-leftRpm);
        rightPower = rightController.calculate(goalRpm-rightRpm);

        leftPower = Math.max(0.0, Math.min(1.0, leftPower));
        rightPower = Math.max(0.0, Math.min(1.0, rightPower));

        leftLauncher.setPower(leftPower);
        rightLauncher.setPower(rightPower);


    }

    public void updatePower() {
        leftLauncher.setPower(power);
        rightLauncher.setPower(power);
    }

}
