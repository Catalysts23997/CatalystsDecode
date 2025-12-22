package org.firstinspires.ftc.teamcode.Competition_Code.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.PIDController;
import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.PIDControllerVelocity;
import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.PIDParams;

public class Launcher {

    double goalRpm;
    public double leftRpm;
    public double rightRpm;

    /// The one motor that controls the launcher. There may be another one added in the
    /// future.
    public DcMotorEx leftLauncher;
    public DcMotorEx rightLauncher;

    public double leftPower = 0;
    public double rightPower = 0;
    double power = 0;

    public static double rightMaxRPM = 6300;
    public static double leftMaxRPM = 6700;


    final double ticksPerRev = 28;
    ElapsedTime timer = new ElapsedTime();

    PIDParams leftPidParams = new PIDParams(0.0015,0,0.00012,1/leftMaxRPM);
    PIDControllerVelocity leftController = new PIDControllerVelocity(leftPidParams);


    PIDParams rightPidParams = new PIDParams(0.0015,0,0.00012,1/rightMaxRPM);
    PIDControllerVelocity rightController = new PIDControllerVelocity(rightPidParams);


    /// Declare a new instance of the launcher system.
    ///
    /// # Only one instance should be active at a given time!
    public Launcher(HardwareMap hardwareMap) {
        leftLauncher = hardwareMap.get(DcMotorEx.class, "leftLauncher");
        // Make sure we know the default state of our motor
        leftLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
        leftLauncher.setPower(0);
        rightLauncher = hardwareMap.get(DcMotorEx.class, "rightLauncher");
        // Make sure we know the default state of our motor
        rightLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLauncher.setPower(0);
        timer.reset();

        leftLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

    public void setLeftPidParams(PIDParams params) {
        leftPidParams = params;
        leftController.setPID(leftPidParams);
    }

    public void setRightPidParams(PIDParams params) {
        rightPidParams = params;
        rightController.setPID(rightPidParams);
    }

    public void setRPM(double rpm) {
        // Clamp the proportion between 0.0 and 1.0
        stop = false;
        this.goalRpm = rpm;
    }

    public void setSpeed(double power) {
        // Clamp the proportion between 0.0 and 1.0
        this.power = power;
    }

    public double getSpeed() {
        return this.power;
    }

    public static double getMotorRpm(DcMotor dcMotor) {
        DcMotorEx motor = (DcMotorEx) dcMotor;

        // (velocity / TICK_PER_ROTATION) * 60

        return (motor.getVelocity() / 28) * 60;
    }

    public double getLeftRpm() {
        return leftRpm;
    }

    public double getRightRpm() {
        return rightRpm;
    }

    boolean stop;
    /// Stop the launcher!
    public void stop() {
        power = 0;
        leftPower = 0;
        rightPower = 0;
        stop = true;
    }


    public void updatePID(){
        double dt = timer.seconds();
        if (dt < 0.05) return;  // updatePID every 50ms
        timer.reset();

        leftRpm = 60 /ticksPerRev * leftLauncher.getVelocity();
        rightRpm = 60 /ticksPerRev * rightLauncher.getVelocity();

        leftPower = leftController.calculate(goalRpm, leftRpm);
        rightPower = rightController.calculate(goalRpm, rightRpm);

        leftPower = Math.max(0.0, Math.min(1.0, leftPower));
        rightPower = Math.max(0.0, Math.min(1.0, rightPower));

        if(!stop) {
            leftLauncher.setPower(leftPower);
            rightLauncher.setPower(rightPower);
        }
        else {
            leftLauncher.setPower(0);
            rightLauncher.setPower(0);
        }
    }

    public void update() {

        leftRpm = 60 /ticksPerRev * leftLauncher.getVelocity();
        rightRpm = 60 /ticksPerRev * rightLauncher.getVelocity();

        if (leftLauncher.getPower() >= .5 && (leftRpm < 175 || rightRpm<175)) {
            leftLauncher.setPower(1.0);
            rightLauncher.setPower(1.0);
        }else{
            leftLauncher.setPower(power);
            rightLauncher.setPower(power);
        }

        leftLauncher.setPower(power);
        rightLauncher.setPower(power);
    }

}
