package org.firstinspires.ftc.teamcode.Competition_Code.Tele.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Launcher;

@Disabled
@TeleOp(name = "LauncherRpmTest", group = "LinearOpMode")
public class LauncherRpmTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Launcher launcher = new Launcher(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Current Left Position: ", launcher.leftLauncher.getCurrentPosition());
            telemetry.addData("Current Right Position: ", launcher.rightLauncher.getCurrentPosition());
            telemetry.addData("Current Left RPM: ", Launcher.getMotorRpm(launcher.leftLauncher));
            telemetry.addData("Current Right RPM: ", Launcher.getMotorRpm(launcher.rightLauncher));
            telemetry.update();
        }
    }
}
