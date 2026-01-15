package org.firstinspires.ftc.teamcode.Competition_Code.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;


public class Servo {

    public com.qualcomm.robotcore.hardware.Servo servo;
    public State state =State.RESET;
    public double launchpos = 0.07;

    public Servo(HardwareMap hardwareMap, String name){
        servo = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, name);
    }

    public void update() {
        if (state == State.RESET) {
            servo.setPosition(launchpos);
        }
        else {
            servo.setPosition(state.servoPos);
        }
    }

    public enum State {
        STOP1(0.58),
        STOP2(0.27),
        RESET(0.0);
        public final double servoPos;
        State(double servoPos) {
            this.servoPos = servoPos;
        }
    }


}
