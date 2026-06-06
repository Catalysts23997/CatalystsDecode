package org.firstinspires.ftc.teamcode.Competition_Code.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.PIDControllerVelocity;
import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.PIDParams;

public class Kickstand {


    /// The one motor that controls the launcher. There may be another one added in the
    /// future.
    public DcMotorEx kickstand;

    public double power = 0;

    static final double ticksPerRev = 28.0;

    Mode mode = Mode.OFF;


    /// Declare a new instance of the launcher system.
    ///
    /// # Only one instance should be active at a given time!
    public Kickstand(HardwareMap hardwareMap) {
        kickstand = hardwareMap.get(DcMotorEx.class, "kickstand");

        kickstand.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        kickstand.setTargetPosition(0);
        kickstand.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        kickstand.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setMode(Mode mode) {
        this.mode = mode;
    }

    public void update() {
        kickstand.setTargetPosition(mode.pos);
        kickstand.setPower(1.0);
    }

    public enum Mode {
        ON(600),
        OFF(0);

        final int pos;

        Mode(int pos) {
            this.pos = pos;
        }
    }
}