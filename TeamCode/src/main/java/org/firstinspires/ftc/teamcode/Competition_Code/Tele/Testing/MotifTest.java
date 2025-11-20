package org.firstinspires.ftc.teamcode.Competition_Code.Tele.Testing;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.AprilTag;

@Disabled
@TeleOp(name = "MotifTest", group = "Linear OpMode")
public class MotifTest extends LinearOpMode {

    AprilTag aprilTag;

    @Override public void runOpMode() {
        aprilTag = new AprilTag(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.a){
                aprilTag.setState(AprilTag.State.On);
                telemetry.addData("motif", aprilTag.getMotif());
                telemetry.update();
            }
            else {
                aprilTag.setState(AprilTag.State.Off);
            }

        }
    }
}
