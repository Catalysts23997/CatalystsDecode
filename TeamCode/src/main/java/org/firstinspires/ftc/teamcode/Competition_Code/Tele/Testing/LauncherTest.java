package org.firstinspires.ftc.teamcode.Competition_Code.Tele.Testing;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Launcher;

@Disabled
@TeleOp(name = "LauncherTEst", group = "LinearOpMode")
public class LauncherTest extends LinearOpMode {
    Launcher launcher;
    @Override
    public void runOpMode() {
        launcher = new Launcher(hardwareMap);
        double speed = 0;
        ElapsedTime timer = new ElapsedTime();
        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.dpad_up && timer.seconds()>=0.5){
                speed += 0.1;
                timer.reset();
            }
            if (gamepad1.dpad_down && timer.seconds()>=0.5){
                speed -= 0.1;
                timer.reset();

            }
            if(gamepad1.a){
                speed = 1;
            }
            if(gamepad1.b){
                speed = 0;
            }

            launcher.setSpeed(speed);
            launcher.update();
        }
    }
}
