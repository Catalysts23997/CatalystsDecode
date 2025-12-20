package org.firstinspires.ftc.teamcode.Competition_Code.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;


public class Servo {

    public com.qualcomm.robotcore.hardware.Servo servo;
    public State state =State.RESET;
    public static double launchpos = 0.12;

    public Servo(HardwareMap hardwareMap, String name){
        servo = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, name);
    }

    public void update(){
        servo.setPosition(state.servoPos);
    }

    public enum State {
        HOLD(0.2),
        Block(0.29),
        LAUNCH(launchpos),
        RESET(0.125),
        STOP(0.55)
        ;
        public final double servoPos;
        State(double servoPos) {
            this.servoPos = servoPos;
        }
    }


}
