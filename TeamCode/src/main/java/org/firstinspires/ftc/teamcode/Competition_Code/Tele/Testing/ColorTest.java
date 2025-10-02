package org.firstinspires.ftc.teamcode.Competition_Code.Tele.Testing;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.ColorSensors;

@TeleOp(name = "ColorTest", group = "Linear OpMode")
public class ColorTest extends LinearOpMode {

    ColorSensors colorSensor;

    @Override
    public void runOpMode() {

        colorSensor = new ColorSensors(hardwareMap, "sensor1");

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Is Something?", colorSensor.checkForRecognition());
            telemetry.addData("Is Green?", colorSensor.isGreen());
            telemetry.addData("Is Purple?", colorSensor.isPurple());
            telemetry.addData("hue", colorSensor.getHue());
            telemetry.update();
        }
    }
}
