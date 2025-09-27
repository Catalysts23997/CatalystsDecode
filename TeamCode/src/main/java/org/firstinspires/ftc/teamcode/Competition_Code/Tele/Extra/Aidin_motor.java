package org.firstinspires.ftc.teamcode.Competition_Code.Tele.Extra;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Aidin_servo", group = "Linear OpMode")
public class Aidin_motor extends LinearOpMode {
    DcMotor Intake1;
    DcMotor Intake2;
    int speed = 0;

    @Override
    public void runOpMode() {
        Intake1 = hardwareMap.get(DcMotor.class, "Intake");
        Intake2 = hardwareMap.get(DcMotor.class, "Intake");


        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.a){
                Intake1.setPower(0.5);
                Intake2.setPower(-0.5);
            }
            else if(gamepad1.b){
                Intake1.setPower(-0.5);
                Intake2.setPower(0.5);
            }
            else{
                Intake1.setPower(0);
                Intake2.setPower(0);
            }


        }
    }
}
