package org.firstinspires.ftc.teamcode.Competition_Code.Tele.Testing;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Pulley;


@TeleOp(name = "PulleyTest", group = "LinearOpMode")
public class PulleyTest extends LinearOpMode {
    Pulley pulley;
    @Override
    public void runOpMode() {
        pulley = new Pulley(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.a){
                pulley.state = Pulley.State.On;
            }
            else {
                pulley.state= Pulley.State.Off;
            }
        }
    }
}
