package org.firstinspires.ftc.teamcode.Competition_Code.Tele.Extra;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@TeleOp(name = "SingleMotorTest")
public class SingleMotorTest extends LinearOpMode {
    DcMotorEx motor;
    double power = 0;
    @Override
    public void runOpMode() {
        ElapsedTime timer = new ElapsedTime();

        motor = hardwareMap.get(DcMotorEx.class, "leftLauncher");
        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.dpad_up && timer.milliseconds() > 60){
                power += 0.05;
                timer.reset();
            }
            else if(gamepad1.dpad_down  && timer.milliseconds() > 60){
                power -= 0.05;
                timer.reset();
            }
            if (power < 0){
                power = 0;
            }
            else if (power > 1){
                power = 1;
            }
            motor.setPower(power);
            telemetry.addData("power", power);
            telemetry.update();

        }
    }
}
