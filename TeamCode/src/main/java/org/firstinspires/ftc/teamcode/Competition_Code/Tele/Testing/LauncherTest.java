package org.firstinspires.ftc.teamcode.Competition_Code.Tele.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Launcher;

@Disabled
@TeleOp(name = "LauncherTEst", group = "LinearOpMode")
public class LauncherTest extends LinearOpMode {
    Launcher launcher;
    private Servo servo;
    private double position;

    @Override
    public void runOpMode() {
        launcher = new Launcher(hardwareMap);
        telemetry = FtcDashboard.getInstance().getTelemetry();

        double speed = 0;
        ElapsedTime timer = new ElapsedTime();

//        servo = hardwareMap.get(Servo.class, "holder");
        waitForStart();


//        position = servo.getPosition();

        while (opModeIsActive()) {

            if (gamepad1.dpad_up && timer.milliseconds() >=100){
                speed += 0.1;
                timer.reset();
            }
            if (gamepad1.dpad_down&& timer.milliseconds() >=100){
                speed -= 0.1;
                timer.reset();

            }
            if(gamepad1.a){
                speed = 1;
            }
            if(gamepad1.b){
                speed = 0;
            }
//            if(gamepad1.x){
//                position = 0.15;
//            }
//            if(gamepad1.y){
//                position = 0.53;
//            }
//
//            if (timer.milliseconds() > 100) {
//                if (gamepad1.dpad_left) {
//                    position -= 0.01;
//                }
//
//                if (gamepad1.dpad_right) {
//                    position += 0.01;
//                }
//
//                timer.reset();
//            }
            launcher.setSpeed(speed);
            launcher.update();

//            servo.setPosition(position);
//
//            telemetry.addData("Servo position: ", position);
            telemetry.addData("power ", speed);
            telemetry.addData("leftrpm", launcher.getLeftRpm());
            telemetry.addData("rightrpm", launcher.getRightRpm());


            telemetry.update();
        }
    }
}