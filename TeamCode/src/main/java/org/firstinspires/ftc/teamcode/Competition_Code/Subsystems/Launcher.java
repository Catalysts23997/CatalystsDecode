package org.firstinspires.ftc.teamcode.Competition_Code.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.PIDControllerVelocity;
import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.PIDParams;

public class Launcher {

    double goalRpm;
    double goalTps;
    public double leftRpm;
    public double rightRpm;

    /// The one motor that controls the launcher. There may be another one added in the
    /// future.
    public DcMotorEx leftLauncher;
    public DcMotorEx rightLauncher;

    public double leftPower = 0;
    public double rightPower = 0;
    double power = 0;

    static final double ticksPerRev = 28.0;

    public static double rightMaxRPM = 6100;
    public static double leftMaxRPM = 6500;

    public static double rightMaxTPS = rightMaxRPM/60.0 * ticksPerRev;
    public static double leftMaxTPS = leftMaxRPM/60.0 * ticksPerRev;

    Mode mode = Mode.POWER;

    public double baseRPM = 2000;

    ElapsedTime timer = new ElapsedTime();

    PIDParams leftPidParams = new PIDParams(0.00365,0,0.000105,1.0/leftMaxRPM);
    PIDControllerVelocity leftController = new PIDControllerVelocity(leftPidParams);


    PIDParams rightPidParams = new PIDParams(0.00365,0,0.000105,1.0/rightMaxRPM);
    PIDControllerVelocity rightController = new PIDControllerVelocity(rightPidParams);

    VoltageSensor voltageSensor;

//    PIDFCoefficients leftCoefficients = new PIDFCoefficients(105.0, 0.0, 8.4, 32767.0/leftMaxTPS);
//    PIDFCoefficients rightCoefficients = new PIDFCoefficients(105.0, 0.0, 8.4, 32767.0/rightMaxTPS);

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

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        leftController.setPID(leftPidParams);
        rightController.setPID(rightPidParams);

//        leftCoefficients.algorithm = MotorControlAlgorithm.PIDF;
//        rightCoefficients.algorithm = MotorControlAlgorithm.PIDF;
//
//        leftLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, leftCoefficients);
//        rightLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, rightCoefficients);
    }

    public void setLeftPidParams(PIDParams params) {
        leftPidParams = params;

//        leftCoefficients = new PIDFCoefficients(params.getKp(),params.getKi(), params.getKd(), params.getKf());

        leftController.setPID(leftPidParams);
//        leftLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, leftCoefficients);
    }

    public void setRightPidParams(PIDParams params) {
        rightPidParams = params;

//        rightCoefficients = new PIDFCoefficients(params.getKp(),params.getKi(), params.getKd(), params.getKf());

        rightController.setPID(rightPidParams);
//        rightLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, rightCoefficients);
    }

    public double getGoalRPM() {
        return goalRpm;
    }

    public void start() {
        mode = Mode.VELOCITY;
    }

    public void setSpeed(double power) {
        // Clamp the proportion between 0.0 and 1.0
        this.power = power;
        mode = Mode.POWER;
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

    /// Stop the launcher!
    public void stop() {
        power = 0;
        leftPower = 0;
        rightPower = 0;
        mode = Mode.POWER;
    }

    public void reverse() {
        power = -0.5;
        mode = Mode.POWER;
    }

    public boolean atTargetRPM(double target, double tolerance) {
        return Math.abs(getLeftRpm() - target) <= tolerance &&
                Math.abs(getRightRpm() - target) <= tolerance;
    }


    public void updatePID(){
        double voltage = voltageSensor.getVoltage();
        leftPower = leftController.calculate(goalRpm, leftRpm, voltage);
        rightPower = rightController.calculate(goalRpm, rightRpm, voltage);

        leftPower = Math.max(-1.0, Math.min(1.0, leftPower));
        rightPower = Math.max(-1.0, Math.min(1.0, rightPower));


        leftLauncher.setPower(leftPower);
        rightLauncher.setPower(rightPower);
    }

//    public void updateVelocity(){
//
//        leftLauncher.setVelocity(goalTps);
//        rightLauncher.setVelocity(goalTps);
//    }

    public int change = 0;
    double lastRPM;

    public void update(){
        leftRpm = 60.0 /ticksPerRev * leftLauncher.getVelocity();
        rightRpm = 60.0 /ticksPerRev * rightLauncher.getVelocity();

        goalRpm = baseRPM + change;
        goalTps = (goalRpm/60.0)*ticksPerRev;

        if(lastRPM!=goalRpm){
            leftController.reset();
            rightController.reset();
        }

        lastRPM = goalRpm;


        switch (mode){
            case VELOCITY: updatePID(); break;
            case POWER: updatePower(); break;
        }
    }

    public void updatePower() {
        leftLauncher.setPower(power);
        rightLauncher.setPower(power);
    }

    enum Mode {
        POWER,
        VELOCITY
    }
}
