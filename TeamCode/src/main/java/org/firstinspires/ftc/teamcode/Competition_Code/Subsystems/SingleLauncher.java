package org.firstinspires.ftc.teamcode.Competition_Code.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.PIDControllerVelocity;
import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.PIDParams;

public class SingleLauncher {

    double goalRpm;
    double goalTps;

    public double rpm;

    /// The one motor that controls the launcher. There may be another one added in the
    /// future.
    public DcMotorEx launcher;

    public double power = 0;

    static final double ticksPerRev = 28.0;

    public static double maxRPM;

    public static double maxTPS = maxRPM/60.0 * ticksPerRev;


    Mode mode = Mode.POWER;

    public double baseRPM = 3600;

    ElapsedTime timer = new ElapsedTime();

    PIDParams pidParams = new PIDParams(0.00365,0,0.000105,1.0/maxRPM);
    PIDControllerVelocity controller = new PIDControllerVelocity(pidParams);

    VoltageSensor voltageSensor;

    /// Declare a new instance of the launcher system.
    ///
    /// # Only one instance should be active at a given time!
    public SingleLauncher(HardwareMap hardwareMap) {
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setDirection(DcMotorSimple.Direction.FORWARD);

        timer.reset();

        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        controller.setPID(pidParams);
    }

    public void setPidParams(PIDParams params) {
        pidParams = params;

        controller.setPID(params);
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


    public double getRpm() {
        return rpm;
    }


    /// Stop the launcher!
    public void stop() {
        power = 0;
        mode = Mode.POWER;
    }

    public void reverse() {
        power = -0.5;
        mode = Mode.POWER;
    }

    public boolean atTargetRPM(double target, double tolerance) {
        return Math.abs(getRpm() - target) <= tolerance;
    }


    public void updatePID(){
        double voltage = voltageSensor.getVoltage();
        power = controller.calculate(goalRpm, rpm, voltage);

        power = Math.max(-1.0, Math.min(1.0, power));

        launcher.setPower(power);
    }


    public int change = 0;
    double lastRPM;

    public void update(){
        rpm = 60.0 /ticksPerRev * launcher.getVelocity();

        goalRpm = baseRPM + change;
        goalTps = (goalRpm/60.0)*ticksPerRev;

        if(lastRPM!=goalRpm){
            controller.reset();
        }

        lastRPM = goalRpm;


        switch (mode){
            case VELOCITY: updatePID(); break;
            case POWER: updatePower(); break;
        }
    }

    public void updatePower() {
        launcher.setPower(power);
    }

    enum Mode {
        POWER,
        VELOCITY
    }
}
