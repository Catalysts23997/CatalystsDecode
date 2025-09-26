package org.firstinspires.ftc.teamcode.Competition_Code.Tele.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Intake;

@TeleOp(name = "Testing Intake Subsystem", group = "Linear OpMode")
public class TestingIntakeSubsystem extends LinearOpMode {

    /// The instance of our Intake subsystem
    public Intake intake;

    /// The main function of the opmode
    @Override
    public void runOpMode() throws InterruptedException {
        // Create a new instance of the Intake subsystem
        intake = new Intake(hardwareMap);

        // Wait for the user to start the opmode
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                // If A is pressed, it goes forward
                intake.state = Intake.State.INTAKING;
            } else if (gamepad1.b) {
                // If B is pressed, it goes backwards
                intake.state = Intake.State.REVERSE;
            } else {
                // If not button is pressed, stop the intake motor
                intake.state = Intake.State.STOPPED;
            }

            // Tick the intake subsystem
            intake.update();
        }
    }
}
