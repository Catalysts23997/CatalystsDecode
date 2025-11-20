package org.firstinspires.ftc.teamcode.Competition_Code.Tele.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Competition_Code.PinpointLocalizer.Localizer;
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.Poses;

import java.util.ArrayList;
import java.util.Arrays;

@Disabled
@TeleOp(name = "IntakeAndDrive", group = "Linear OpMode")
public class IntakeAndDrive extends LinearOpMode {

    /// The instance of our Intake subsystem
    public Intake intake;

    public Drivetrain drive;

    /// The main function of the opmode
    @Override
    public void runOpMode() throws InterruptedException {
        // Create a new instance of the Intake subsystem
        intake = new Intake(hardwareMap);
        drive = new Drivetrain(hardwareMap);

        Localizer localizer = new Localizer(hardwareMap, new Poses(0.0, 0.0, 0.0));
        Drivetrain drive = new Drivetrain(hardwareMap);

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

            localizer.update(false);

            if(gamepad1.y){
                localizer.resetHeading();
            }

            drive.update
                    (
                            new ArrayList<>(Arrays.asList(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x))
                    );


            // Tick the intake subsystem
            intake.update();
        }
    }
}
