package org.firstinspires.ftc.teamcode.Competition_Code.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.AprilTag;
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.ColorSensors;
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Pulley;
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Servo;
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Intake.State;

public class Comp1Actions {
    AprilTag aprilTag;
    public Servo servo;
    public ColorSensors ball1;
    public ColorSensors ball2;

    Intake intake;
    Launcher launcher;

    Pulley pulley;

    //apriltag will dictate the motif number
    public int motif;

    Telemetry telemetry;

    //TODO() Need to make motor subsystem, find Servo values, and get a color sensor to check purple/green. Also make tests


    public void update() {
        aprilTag.update();
        servo.update();
        intake.update();
        launcher.update();
        pulley.update();

        telemetry.addData("Intake State", intake.state);
        telemetry.addData("Pulley State", pulley.state);
        telemetry.addData("Servo State", servo.state);

        telemetry.addData("Ball1 Is Green?", ball1.isGreen());
        telemetry.addData("Ball1 Is Purple?", ball1.isPurple());
        telemetry.addData("Ball1 Hue?", ball1.getHue());
        telemetry.addData("Ball2 Is Green?", ball2.isGreen());
        telemetry.addData("Ball2 Is Purple?", ball2.isPurple());

        telemetry.update();
    }



    public Comp1Actions(HardwareMap hardwareMap, Telemetry telemetry) {
        aprilTag = new AprilTag(hardwareMap);
        servo = new Servo(hardwareMap, "port0");
        ball1 = new ColorSensors(hardwareMap, "sensor1");
        launcher = new Launcher(hardwareMap);
        intake = new Intake(hardwareMap);
        pulley = new Pulley(hardwareMap);
        ball2 = new ColorSensors(hardwareMap, "sensor2");
        this.telemetry = telemetry;
    }

    /// This is the timeout that our sensors will use. If
    /// a sensor doesn't detect something in this amount
    /// of time, it will stop searching.
    double cameraTimeout = 2000;

    public Action CheckMotif() {
        return new Action() {

            final ElapsedTime timer = new ElapsedTime();
            boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                aprilTag.setState(AprilTag.State.On);
                if (!initialized) {
                    timer.reset();
                    initialized = true;
                }

                // Check the current motif
                motif = aprilTag.getMotif();
                telemetry.addData("AprilTag", "Waiting... " + timer.milliseconds() + "ms");
                // If we don't see anything, stop searching
                return motif == 0 && timer.milliseconds() <= cameraTimeout;
            }
        };
    }
    public Action OffCamera() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                aprilTag.setState(AprilTag.State.Off);

                return false;
            }
        };
    }

    //intake stuff
    double colorTimeout = 2000;

    public Action Ball1Check() {
        return new Action() {

            final ElapsedTime timer = new ElapsedTime();
            boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    timer.reset();
                    initialized = true;
                }

                telemetry.addData("Ball1Check", "Waiting... " + timer.milliseconds() + "ms");

                // If we don't see anything, stop checking
                return !(ball1.isGreen() || ball1.isPurple()) &&
                        timer.milliseconds() <= colorTimeout;
            }
        };
    }

    public Action Ball2Check() {
        return new Action() {

            final ElapsedTime timer = new ElapsedTime();
            boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    timer.reset();
                    initialized = true;
                }

                telemetry.addData("Ball2Check", "Waiting... " + timer.milliseconds() + "ms");

                // If we don't see anything, stop checking
                return !(ball2.isGreen() || ball2.isPurple()) &&
                        timer.milliseconds() <= colorTimeout;
            }
        };
    }

    public Action StartIntake = new Action() {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            // Turn on our intake system!
            intake.state = State.INTAKING;
            pulley.state = Pulley.State.On;

            // We return false because this only has to run once
            return false;
        }
    };

    public Action StopIntake = new Action() {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            // Stop our intake system
            intake.state = State.STOPPED;
            pulley.state = Pulley.State.Off;

            // We return false because this only has to run once
            return false;
        }
    };

    public Action HoldBall = new Action() {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            servo.state = Servo.State.HOLD;

            // We return false because this only has to run once
            return false;
        }
    };

    // These functions are for autonomous. Even if we don't find
    // anything, we will just move on
    public SequentialAction Ball1() {return new SequentialAction(StartIntake, Ball1Check(),WaitAction(200), HoldBall);}
    public SequentialAction Ball2() {return new SequentialAction(StartIntake, Ball2Check());}
    public SequentialAction Ball3() {return  new SequentialAction(StartIntake, WaitAction(1000), StopIntake);}

    public SequentialAction Balls(){return new SequentialAction(Ball1(), Ball2(), Ball3());}

    //shooting stuff
    double shootTime = 600;       // ms for servo launch duration
    double detectTimeout = 2000;   // ms to wait for ball detection
    double speedUpTime = 2000;     // time for flywheel to reach speed
    double shootingInterval = 500;

    public Action WaitAction(double waitMs) {

        return new Action() {

            private final ElapsedTime timer = new ElapsedTime();
            boolean initialized = false;


            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // initialize timer on first run
                if (!initialized) {
                    initialized =true;
                    timer.reset();
                }

                // return true until wait time is reached
                return timer.milliseconds() < waitMs;
            }
        };
    }

    public Action Shoot() {
        return new Action() {
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
                    telemetry.addData("Shoot", "Initialized — waiting for ball");
                }

                // Wait for ball
                if (waitingForBall) {
                    servo.state = Servo.State.LAUNCH;
                    timer.reset(); // restart timer for launch duration
                    waitingForBall = false;
                    launched = true;
                    telemetry.addData("Shoot", "Ball detected — launching");
                }

                // Launch phase
                if (launched) {
                    if (timer.milliseconds() >= shootTime) {
                        servo.state = Servo.State.RESET;
                        telemetry.addData("Shoot", "Shot complete — servo reset");
                        return false;
                    } else {
                        telemetry.addData("Shoot", "Launching... " + timer.milliseconds() + "ms");
                        return true;
                    }
                }

                return true;
            }
        };
    }

    public Action StartShooter = new Action() {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            launcher.setSpeed(0.67);

            // We return false because this only has to run once
            return false;
        }

    };
    public Action Reverse = new Action() {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            pulley.state = Pulley.State.Reverse;

            // We return false because this only has to run once
            return false;
        }

    };
    public Action Off = new Action() {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            pulley.state = Pulley.State.Off;

            // We return false because this only has to run once
            return false;
        }

    };


    public Action StopShooter = new Action() {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            launcher.stop();

            // We return false because this only has to run once
            return false;
        }
    };

    public Action Cycle() {
        return new Action() {
            final ElapsedTime timer = new ElapsedTime();
            boolean initialized = false;
            boolean started = false;

            final double detectTimeout = 2000;     // ms to wait for ball2 to appear
            final double cycleTimeout = 2000;      // ms to run pulley before forcing stop

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                // Phase 1: Wait for ball2 (the start signal)
                if (!initialized) {
                    timer.reset();
                    pulley.state = Pulley.State.Slow;
                    servo.state = Servo.State.RESET;
                    intake.state = State.INTAKING;
                    initialized = true;
                    telemetry.addData("Cycle", "Waiting for ball2...");
                }

                if (!started) {
                    // If ball2 detected → start cycle
                    if ((ball2.isGreen() || ball2.isPurple())) {
                        timer.reset(); // restart timer for next phase
                        pulley.state = Pulley.State.Slow;

                        started = true;
                        telemetry.addData("Cycle", "Ball2 detected → starting pulley");
                    }
                    // If timeout before detection → abort
                    else if (timer.milliseconds() > detectTimeout) {
                        pulley.state = Pulley.State.Off;
                        intake.state = State.STOPPED;

                        telemetry.addData("Cycle", "No ball2 detected → skipping cycle");
                        return false;
                    }
                    // Still waiting
                    else {
                        telemetry.addData("Cycle", "Waiting for ball2... " + timer.milliseconds() + "ms");
                        return true;
                    }
                }

                // Phase 2: Running until ball1 detected or timeout
                if (started) {
                    // If ball1 detected → stop and hold
                    if ((ball1.isGreen() || ball1.isPurple())) {
                        pulley.state = Pulley.State.Reverse;
                        intake.state = State.STOPPED;
                        telemetry.addData("Cycle", "Ball1 detected → stopping");
                        return false;
                    }

                    // If ran too long → stop
                    if (timer.milliseconds() > cycleTimeout) {
                        pulley.state = Pulley.State.Off;
                        intake.state = State.STOPPED;
                        servo.state = Servo.State.HOLD;
                        telemetry.addData("Cycle", "Timeout → stopping");
                        return false;
                    }

                    // Otherwise keep going
                    telemetry.addData("Cycle", "Running... " + timer.milliseconds() + "ms");
                    return true;
                }

                return true;
            }
        };
    }

    public Action CycleShoot() {
        return new Action() {
            final ElapsedTime timer = new ElapsedTime();
            boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                // Phase 1: Wait for ball2 (the start signal)
                if (!initialized) {
                    timer.reset();
                    pulley.state = Pulley.State.On;
                    intake.state = State.INTAKING;
                    initialized = true;
                }

                if(timer.milliseconds()>=5000){
                    pulley.state = Pulley.State.Off;
                    intake.state = State.STOPPED;
                    return false;
                }
                return true;
            }
        };
    }



    public SequentialAction Shoot3Balls() {
        return new SequentialAction(
                StartShooter,
                WaitAction(speedUpTime),
                Shoot(),
                Cycle(),
                WaitAction(shootingInterval),
                HoldBall,
                Off,
                Shoot(),
                Cycle(),
                HoldBall,
                Off,
                Shoot(),
                StopShooter
        );
    }
    public SequentialAction AutoShoot() {
        return new SequentialAction(
                Shoot(),
                Cycle(),
                WaitAction(shootingInterval),
                HoldBall,
                Off,
                Shoot(),
                Cycle(),
                HoldBall,
                Off,
                Shoot(),
                StopShooter
        );
    }

    public SequentialAction Shoot2Balls() {
        return new SequentialAction(
                StartShooter,
                new SleepAction(speedUpTime / 1000),
                Shoot(),
                Cycle(),
                new SleepAction(shootingInterval / 1000),
                Shoot(),
                StopShooter
        );
    }
    public SequentialAction Shoot1Ball() {
        return new SequentialAction(
                StartShooter,
                new SleepAction(speedUpTime / 1000),
                Shoot(),
                StopShooter
        );
    }
}
