package org.firstinspires.ftc.teamcode.Competition_Code.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Pulley {

    DcMotor pulley;
    public State state;

    public Pulley(HardwareMap hardwareMap){
        pulley = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, "pulley");
        pulley.setDirection(DcMotorSimple.Direction.FORWARD);
        state= State.Off;
    }

    public void update(){
        pulley.setPower(state.power);
    }

    public enum State {
        On(1.0),
        Slow(0.9),
        Reverse(-1.0),
        Off(0.0);
        public final double power;
        State(double power) {
            this.power = power;
        }
    }


}
