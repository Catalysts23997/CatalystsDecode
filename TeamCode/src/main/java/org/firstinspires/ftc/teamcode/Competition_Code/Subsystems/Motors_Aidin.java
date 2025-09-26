package org.firstinspires.ftc.teamcode.Competition_Code.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Motors_Aidin {
    public DcMotor Motor1;
    public DcMotor Motor2;
    public State state = State.off;

    public Motors_Aidin(HardwareMap hardwareMap) {
        Motor1 = hardwareMap.get(DcMotor.class, "Motor1");
        Motor2 = hardwareMap.get(DcMotor.class, "Motor2");
    }

    public void update() {
        Motor1.setPower(state.power);
        Motor2.setPower(-state.power);
    }


    public enum State {
        forward(1),
        backward(-1),
        off(0);
        public final double power;
        State(double power) {
            this.power = power;
        }
    }
}
