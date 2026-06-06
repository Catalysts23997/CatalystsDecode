package org.firstinspires.ftc.teamcode.Competition_Code.Tele.Extra;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Kickstand;

@TeleOp(name = "KickstandTest", group = "LinearOpMode")
public class KickstandTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Kickstand kickstand = new Kickstand(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.a){
                kickstand.setMode(Kickstand.Mode.ON);
            }
            else {
                kickstand.setMode(Kickstand.Mode.OFF);

            }
            kickstand.update();
            telemetry.addData("Position", kickstand.kickstand.getCurrentPosition());
            telemetry.addData("Target", kickstand.kickstand.getTargetPosition());
            telemetry.update();
        }
    }
}
