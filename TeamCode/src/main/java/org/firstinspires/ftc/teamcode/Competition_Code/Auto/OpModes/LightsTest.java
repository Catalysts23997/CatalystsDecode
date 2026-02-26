package org.firstinspires.ftc.teamcode.Competition_Code.Auto.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Lights;


@TeleOp(name = "LightsTest")public class LightsTest extends LinearOpMode {
    public Lights lights;

    @Override public void runOpMode() {
        waitForStart();
        while (opModeIsActive()) {

            if (gamepad1.dpad_up){
                lights.color = Lights.Color.green;
            }
            else if (gamepad1.dpad_down){
                lights.color = Lights.Color.red;
            }
            else{
                lights.color = Lights.Color.red;
            }
        }
    }
}
