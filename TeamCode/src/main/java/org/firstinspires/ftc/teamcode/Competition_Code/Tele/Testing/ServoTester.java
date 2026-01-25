package org.firstinspires.ftc.teamcode.Competition_Code.Tele.Testing;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@TeleOp(name = "ServoTester", group = "LinearOpMode")
public class ServoTester extends LinearOpMode {

    private Servo servo;
    private double position;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        servo = hardwareMap.get(Servo.class, "holder");
        position = servo.getPosition();

        ElapsedTime timer = new ElapsedTime();
        TelemetryPacket telemetryPacket = new TelemetryPacket();

        while (opModeIsActive()) {
            if (timer.milliseconds() > 100) {
                if (gamepad1.dpad_up) {
                    position++;
                }

                if (gamepad1.dpad_down) {
                    position++;
                }

                timer.reset();
            }

            servo.setPosition(position);

            telemetryPacket.addLine("Servo position: " + position);
        }
    }
}
