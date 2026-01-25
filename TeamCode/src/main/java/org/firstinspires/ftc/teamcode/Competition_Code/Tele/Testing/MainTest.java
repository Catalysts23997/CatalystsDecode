package org.firstinspires.ftc.teamcode.Competition_Code.Tele.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Competition_Code.AllianceColor;
import org.firstinspires.ftc.teamcode.Competition_Code.PinpointLocalizer.Localizer;
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.Poses;

import java.util.ArrayList;
import java.util.Arrays;
@Disabled
@TeleOp(name = "MainTest", group = "LinearOpMode")
public class MainTest extends LinearOpMode {

    /// The instance of our intake subsystem
    private Intake intake;
    /// The instance of our drive train
    private Drivetrain drive;
    /// The instance of our launcher subsystem
    private Launcher launcher;
    /// THe speed of our launcher
    private double speed = 0.0;

    @Override
    public void runOpMode() {
        // Get the handles to our subsystems!
        intake = new Intake(hardwareMap);
        drive = new Drivetrain(hardwareMap, AllianceColor.Blue);
        launcher = new Launcher(hardwareMap);

        Localizer localizer = new Localizer(hardwareMap, new Poses(0.0, 0.0, 0.0));
        ElapsedTime timer = new ElapsedTime();

        // Wait for the user to start the opmode
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                // If A is pressed, it goes forward
                intake.state = Intake.State.INTAKING;
            } else if (gamepad2.b) {
                // If B is pressed, it goes backwards
                intake.state = Intake.State.REVERSE;
            } else {
                // If not button is pressed, ball2 the intake motor
                intake.state = Intake.State.STOPPED;
            }

            localizer.update();

            if(gamepad1.y){
                localizer.resetOdo();
            }

            drive.update(
                    new ArrayList<>(
                            Arrays.asList(
                                    gamepad1.left_stick_x,
                                    -gamepad1.left_stick_y,
                                    gamepad1.right_stick_x
                            )
                    )
            );


            // Tick the intake subsystem
            intake.update();

            // Now that we have undated our drivetrain and intake, we
            // can how update the launcher
            if (gamepad1.dpad_up && timer.seconds()>=0.5){
                speed += 0.1;
                timer.reset();
            }
            if (gamepad1.dpad_down && timer.seconds()>=0.5){
                speed -= 0.1;
                timer.reset();

            }
            if(gamepad1.right_trigger > 0.1){
                speed = 1;
            }
            if(gamepad1.left_trigger > 0.1){
                speed = 0;
            }

            launcher.setSpeed(speed);
            launcher.update();
        }
    }

}