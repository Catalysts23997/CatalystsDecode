package org.firstinspires.ftc.teamcode.Competition_Code.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.AprilTag;
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.ColorSensors;
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Pulley;
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Servo;
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Intake.State;

public class Comp1Actions {
    AprilTag aprilTag;
    Servo servo;
    public ColorSensors ball1;
    public ColorSensors ball2;

    Intake intake;
    Launcher launcher;

    Pulley pulley;

    //apriltag will dictate the motif number
    public int motif;

    //TODO() Need to make motor subsystem, find Servo values, and get a color sensor to check purple/green. Also make tests


    public void update() {
        aprilTag.update();
        servo.update();
        intake.update();
        launcher.update();
        pulley.update();
    }



    public Comp1Actions(HardwareMap hardwareMap) {
        aprilTag = new AprilTag(hardwareMap);
        servo = new Servo(hardwareMap, "port1");
        ball1 = new ColorSensors(hardwareMap, "sensor1");
        launcher = new Launcher(hardwareMap);
        intake = new Intake(hardwareMap);
        pulley = new Pulley(hardwareMap);
        ball2 = new ColorSensors(hardwareMap, "sensor2");
    }

    double timeoutMilliseconds = 1000;

    public Action CheckMotif = new Action() {

        final ElapsedTime timer = new ElapsedTime();
        boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                timer.reset();
                initialized = true;
            }

            motif = aprilTag.getMotif();


            return motif == 0 && timer.milliseconds()<=timeoutMilliseconds;
        }
    };

    //intake stuff

    public Action Ball1Check = new Action() {

        final ElapsedTime timer = new ElapsedTime();
        boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                timer.reset();
                initialized = true;

            }



            return !ball1.checkForRecognition() && timer.milliseconds()<=timeoutMilliseconds;
        }
    };

    public Action Ball2Check = new Action() {

        final ElapsedTime timer = new ElapsedTime();
        boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                timer.reset();
                initialized = true;
            }

            return !ball2.checkForRecognition() && timer.milliseconds()<=timeoutMilliseconds;
        }
    };

    public Action StartIntake = new Action() {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.state = State.INTAKING;
            pulley.state = Pulley.State.On;


            return false;
        }
    };

    public Action StopIntake = new Action() {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.state = State.STOPPED;
            pulley.state = Pulley.State.Off;

            return false;
        }
    };

    public Action HoldBall = new Action() {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            servo.state = Servo.State.HOLD;

            return false;
        }
    };

    public SequentialAction Ball1 = new SequentialAction(StartIntake, Ball1Check, new SleepAction(0.1), HoldBall);
    public SequentialAction Ball2 = new SequentialAction(StartIntake, Ball2Check);
    public SequentialAction Ball3 = new SequentialAction(StartIntake, new SleepAction(0.5), StopIntake);

    //shooting stuff

    double shootTime = 300;       // ms for servo launch duration
    double detectTimeout = 500;   // ms to wait for ball detection
    double speedUpTime = 800;     // time for flywheel to reach speed
    double shootingInterval = 200;

    public Action Shoot = new Action() {
        final ElapsedTime timer = new ElapsedTime();
        boolean initialized = false;
        boolean waitingForBall = true;
        boolean launched = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // Reset timer when action starts
            if (!initialized) {
                timer.reset();
                initialized = true;
                packet.put("Shoot", "Initialized — waiting for ball");
            }

            // Wait for ball
            if (waitingForBall) {
                if (ball1.checkForRecognition()) {
                    servo.state = Servo.State.LAUNCH;
                    timer.reset(); // restart timer for launch duration
                    waitingForBall = false;
                    launched = true;
                    packet.put("Shoot", "Ball detected — launching");
                } else if (timer.milliseconds() > detectTimeout) {
                    servo.state = Servo.State.RESET;
                    packet.put("Shoot", "No ball detected — skipping shot");
                    return false;
                } else {
                    packet.put("Shoot", "Waiting for ball... " + timer.milliseconds() + "ms");
                    return true;
                }
            }

            // Launch phase
            if (launched) {
                if (timer.milliseconds() >= shootTime) {
                    servo.state = Servo.State.RESET;
                    packet.put("Shoot", "Shot complete — servo reset");
                    return false;
                } else {
                    packet.put("Shoot", "Launching... " + timer.milliseconds() + "ms");
                    return true;
                }
            }

            return false;
        }
    };

    public Action StartShooter = new Action() {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            launcher.setSpeed(0.8);
            return false;


        }
    };

    public Action StopShooter = new Action() {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            launcher.stop();
            return false;
        }
    };

    public Action Cycle = new Action() {
        final ElapsedTime timer = new ElapsedTime();
        boolean initialized = false;
        boolean started = false;

        final double detectTimeout = 500;     // ms to wait for ball2 to appear
        final double cycleTimeout = 2000;      // ms to run pulley before forcing stop

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            // Phase 1: Wait for ball2 (the start signal)
            if (!initialized) {
                timer.reset();
                initialized = true;
                packet.put("Cycle", "Waiting for ball2...");
            }

            if (!started) {
                // If ball2 detected → start cycle
                if (ball2.checkForRecognition()) {
                    pulley.state = Pulley.State.On;
                    timer.reset(); // restart timer for next phase
                    started = true;
                    packet.put("Cycle", "Ball2 detected → starting pulley");
                }
                // If timeout before detection → abort
                else if (timer.milliseconds() > detectTimeout) {
                    packet.put("Cycle", "No ball2 detected → skipping cycle");
                    return false;
                }
                // Still waiting
                else {
                    packet.put("Cycle", "Waiting for ball2... " + timer.milliseconds() + "ms");
                    return true;
                }
            }

            // Phase 2: Running until ball1 detected or timeout
            if (started) {
                // If ball1 detected → stop and hold
                if (ball1.checkForRecognition()) {
                    pulley.state = Pulley.State.Off;
                    servo.state = Servo.State.HOLD;
                    packet.put("Cycle", "Ball1 detected → stopping");
                    return false;
                }

                // If ran too long → stop
                if (timer.milliseconds() > cycleTimeout) {
                    pulley.state = Pulley.State.Off;
                    servo.state = Servo.State.HOLD;
                    packet.put("Cycle", "Timeout → stopping");
                    return false;
                }

                // Otherwise keep going
                packet.put("Cycle", "Running... " + timer.milliseconds() + "ms");
                return true;
            }

            return true;
        }
    };


    public SequentialAction ShootBalls = new SequentialAction(
            StartShooter,
            new SleepAction(speedUpTime/1000),
            Shoot,
            Cycle,
            new SleepAction(shootingInterval/1000),
            Shoot,
            Cycle,
            new SleepAction(shootingInterval/1000),
            Shoot,
            StopShooter
    );
}
