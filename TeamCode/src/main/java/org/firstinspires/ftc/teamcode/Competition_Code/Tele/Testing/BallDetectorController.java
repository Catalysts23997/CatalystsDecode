package org.firstinspires.ftc.teamcode.Competition_Code.Tele.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Competition_Code.Auto.BallDetector.BallDetectorGlobals;
import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.Poses;

@TeleOp(name = "BallDetectorController", group = "Linear OpMode")
public class BallDetectorController extends LinearOpMode {
    private final ElapsedTime buttonTimer = new ElapsedTime();
    private int currentLocation = 0;

    @Override
    public void runOpMode() {
        // ensure we are safe to change the values
        if (!BallDetectorGlobals.IS_DEBUG) {
            throw new RuntimeException("This OpMode cannot be used in competition or when debug mode is not enabled!");
        }

        waitForStart();

        while (opModeIsActive()) {
            switch (currentLocation) {
                case 0: telemetry.addLine("Currently modifying WAIT"); break;
                case 1: telemetry.addLine("Currently modifying PICKUP_FIRST"); break;
                case 2: telemetry.addLine("Currently modifying PICKUP_SECOND"); break;
                default:
                    break;
            }

            switch (currentLocation) {
                case 0: telemetry.addLine(BallDetectorGlobals.LOCATION_WAIT.toString()); break;
                case 1: telemetry.addLine(BallDetectorGlobals.LOCATION_PICKUP_FIRST.toString()); break;
                case 2: telemetry.addLine(BallDetectorGlobals.LOCATION_PICKUP_SECOND.toString()); break;
                default:
                    break;
            }

            telemetry.addLine(); // blank line

            // position
            if (buttonTimer.milliseconds() > 150 && gamepad1.dpad_up) {
                buttonTimer.reset();
                switch (currentLocation) {
                    case 0: BallDetectorGlobals.LOCATION_WAIT.pose.add(new Poses(0, -5, 0.0)); break;
                    case 1: BallDetectorGlobals.LOCATION_PICKUP_FIRST.pose.add(new Poses(0, -5, 0.0)); break;
                    case 2: BallDetectorGlobals.LOCATION_PICKUP_SECOND.pose.add(new Poses(0, -5, 0.0)); break;
                    default:
                        break;
                }
            }

            if (buttonTimer.milliseconds() > 150 && gamepad1.dpad_down) {
                buttonTimer.reset();
                switch (currentLocation) {
                    case 0: BallDetectorGlobals.LOCATION_WAIT.pose.add(new Poses(0, 2, 0.0)); break;
                    case 1: BallDetectorGlobals.LOCATION_PICKUP_FIRST.pose.add(new Poses(0, 2, 0.0)); break;
                    case 2: BallDetectorGlobals.LOCATION_PICKUP_SECOND.pose.add(new Poses(0, 2, 0.0)); break;
                    default:
                        break;
                }
            }

            if (buttonTimer.milliseconds() > 150 && gamepad1.dpad_left) {
                buttonTimer.reset();
                switch (currentLocation) {
                    case 0: BallDetectorGlobals.LOCATION_WAIT.pose.add(new Poses(-2, 0, 0.0)); break;
                    case 1: BallDetectorGlobals.LOCATION_PICKUP_FIRST.pose.add(new Poses(-2, 0, 0.0)); break;
                    case 2: BallDetectorGlobals.LOCATION_PICKUP_SECOND.pose.add(new Poses(-2, 0, 0.0)); break;
                    default:
                        break;
                }
            }

            if (buttonTimer.milliseconds() > 150 && gamepad1.dpad_right) {
                buttonTimer.reset();
                switch (currentLocation) {
                    case 0: BallDetectorGlobals.LOCATION_WAIT.pose.add(new Poses(5, 0, 0.0)); break;
                    case 1: BallDetectorGlobals.LOCATION_PICKUP_FIRST.pose.add(new Poses(5, 0, 0.0)); break;
                    case 2: BallDetectorGlobals.LOCATION_PICKUP_SECOND.pose.add(new Poses(5, 0, 0.0)); break;
                    default:
                        break;
                }
            }

            // rotation
            if (buttonTimer.milliseconds() > 150 && gamepad1.left_trigger_pressed) {
                buttonTimer.reset();
                switch (currentLocation) {
                    case 0: BallDetectorGlobals.LOCATION_WAIT.pose.add(new Poses(0, 0, -0.12)); break;
                    case 1: BallDetectorGlobals.LOCATION_PICKUP_FIRST.pose.add(new Poses(0, 0, -0.12)); break;
                    case 2: BallDetectorGlobals.LOCATION_PICKUP_SECOND.pose.add(new Poses(0, 0, -0.12)); break;
                    default:
                        break;
                }
            }

            if (buttonTimer.milliseconds() > 150 && gamepad1.right_trigger_pressed) {
                buttonTimer.reset();
                switch (currentLocation) {
                    case 0: BallDetectorGlobals.LOCATION_WAIT.pose.add(new Poses(0, 0, 0.12)); break;
                    case 1: BallDetectorGlobals.LOCATION_PICKUP_FIRST.pose.add(new Poses(0, 0, 0.12)); break;
                    case 2: BallDetectorGlobals.LOCATION_PICKUP_SECOND.pose.add(new Poses(0, 0, 0.12)); break;
                    default:
                        break;
                }
            }

            // max time
            if (buttonTimer.milliseconds() > 150 && gamepad1.left_stick_y > 0.5f) {
                buttonTimer.reset();
                switch (currentLocation) {
                    case 0: BallDetectorGlobals.LOCATION_WAIT.maxTime -= 15; break;
                    case 1: BallDetectorGlobals.LOCATION_PICKUP_FIRST.maxTime -= 15; break;
                    case 2: BallDetectorGlobals.LOCATION_PICKUP_SECOND.maxTime -= 15; break;
                    default:
                        break;
                }
            }

            if (buttonTimer.milliseconds() > 150 && gamepad1.right_stick_y > 0.5f) {
                buttonTimer.reset();
                switch (currentLocation) {
                    case 0: BallDetectorGlobals.LOCATION_WAIT.maxTime += 15; break;
                    case 1: BallDetectorGlobals.LOCATION_PICKUP_FIRST.maxTime += 15; break;
                    case 2: BallDetectorGlobals.LOCATION_PICKUP_SECOND.maxTime += 15; break;
                    default:
                        break;
                }
            }

            // cycling
            if (gamepad1.leftBumperWasPressed()) {
                currentLocation -= 1;
            }

            if (gamepad1.rightBumperWasPressed()) {
                currentLocation += 1;
            }

            if (gamepad1.a && gamepad1.y) {
                BallDetectorGlobals.resetGlobalsForComp(telemetry);
            }

            currentLocation = currentLocation % 3;

            telemetry.update();
        }
    }
}