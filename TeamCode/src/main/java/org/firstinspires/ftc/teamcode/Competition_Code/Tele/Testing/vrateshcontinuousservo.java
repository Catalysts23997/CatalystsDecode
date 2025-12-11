package org.firstinspires.ftc.teamcode.Competition_Code.Tele.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;
import java.util.List;

@Config
@TeleOp(name = "vrateshcontinuousservo", group = "Linear OpMode")
public class vrateshcontinuousservo extends LinearOpMode {

    public static double servoPower=0.0;
    public static int index=0;




    @Override
    public void runOpMode() {

        List<CRServo> servoList = Arrays.asList(hardwareMap.get(CRServo.class, "axon"));

        waitForStart();
        while (opModeIsActive()){
            servoList.get(index).setPower(servoPower);
        }
    }
}

