package org.firstinspires.ftc.teamcode.Competition_Code.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
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
    public int motif;

    public ColorSensors ball1;
    public ColorSensors ball2;

    public Servo kicker;
    public Servo holder;

    Intake intake;
    Pulley pulley;

    Launcher launcher;

    Telemetry telemetry;

    public void update() {
        aprilTag.update();

        intake.update();
        pulley.update();

        kicker.update();
        holder.update();

        launcher.updatePID();

        telemetry.addData("Ball1 Is Green?", ball1.isGreen());
        telemetry.addData("Ball1 Is Purple?", ball1.isPurple());
        telemetry.addData("Ball1 Hue?", ball1.getHue());
        telemetry.addData("Ball2 Is Green?", ball2.isGreen());
        telemetry.addData("Ball2 Is Purple?", ball2.isPurple());
        telemetry.addData("Ball2 Hue?", ball2.getHue());

        telemetry.addData("Intake State", intake.state);
        telemetry.addData("Pulley State", pulley.state);

        telemetry.addData("Kicker State", kicker.state);
        telemetry.addData("Holder State", holder.state);

        telemetry.addData("Launcher Speed", launcher.getGoalRpm());

        telemetry.update();
    }

    public Comp1Actions(HardwareMap hardwareMap, Telemetry telemetry) {
        aprilTag = new AprilTag(hardwareMap);

        ball1 = new ColorSensors(hardwareMap, "ball1");
        ball2 = new ColorSensors(hardwareMap, "ball2");

        intake = new Intake(hardwareMap);
        pulley = new Pulley(hardwareMap);

        kicker = new Servo(hardwareMap, "kicker");
        holder = new Servo(hardwareMap,"holder");

        launcher = new Launcher(hardwareMap);

        this.telemetry = telemetry;
    }

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

    //camera actions
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

    //color sensor actions
    double colorTimeout = 1000;

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
                return !(ball1.isGreen() || ball1.isPurple()) && timer.milliseconds() <= colorTimeout;
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
                return !(ball2.isGreen() || ball2.isPurple()) && timer.milliseconds() <= colorTimeout;
            }
        };
    }

    //intake
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

    public Action ReverseIntake = new Action() {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            // reverse intake system
            intake.state = State.REVERSE;
            pulley.state = Pulley.State.Reverse;

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

    //servo control

    public Action HoldBall = new Action() {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            holder.state = Servo.State.STOP;

            // We return false because this only has to run once
            return false;
        }
    };


    public Action ReleaseBall = new Action() {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            holder.state = Servo.State.RESET;

            // We return false because this only has to run once
            return false;
        }
    };

    public double ball3Timeout = 800;

    public SequentialAction BallsIntake(){return new SequentialAction(StartIntake, Ball1Check(), Ball2Check(), WaitAction(ball3Timeout), StopIntake);}

    //shooting stuff
    public double launchRpm = 5000;

    double speedUpTime = 2000;      // time for flywheel to reach speed
    double servoShootTime = 700;    // ms for servo launch duration
    double servoReleaseTime = 400;  // ms for servo release
    double shootingInterval = 500;  // ms between shots
    double pulleyShootTime = 2800;  // ms for pulley to shoot 2 balls

    public Action StartShooter = new Action() {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            launcher.setRPM(launchRpm);

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

    public Action Shoot() {
        return new Action() {
            final ElapsedTime timer = new ElapsedTime();
            boolean initialized = false;
            boolean launched = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // Reset timer when action starts
                if (!initialized) {
                    timer.reset();
                    initialized = true;

                    holder.state = Servo.State.RESET;
                    kicker.state = Servo.State.LAUNCH;

                    launched = true;
                    telemetry.addData("Shoot", "Initialized — launching ball");
                }

                // Launch phase
                if (launched) {
                    if (timer.milliseconds() >= servoShootTime) {
                        kicker.state = Servo.State.RESET;

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

    double detectTimeout = 2000;   // ms to wait for ball detection

    public Action Cycle() {
        return new Action() {
            final ElapsedTime timer = new ElapsedTime();
            boolean initialized = false;
            boolean ballFound = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                // Phase 1: Wait for ball2 (the start signal)
                if (!initialized) {
                    timer.reset();

                    pulley.state = Pulley.State.On;
                    intake.state = State.INTAKING;
                    holder.state = Servo.State.STOP;

                    initialized = true;
                    telemetry.addData("Cycle", "Waiting for ball2...");
                }

                if (!ballFound) {
                    // If ball2 detected → start cycle
                    if ((ball2.isGreen() || ball2.isPurple())) {
                        timer.reset(); // restart timer for next phase

                        ballFound = true;
                        telemetry.addData("Cycle", "Ball2 detected");
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
                if (ballFound) {
                    // If ball1 detected → stop and hold
                    if ((ball1.isGreen() || ball1.isPurple())) {
                        pulley.state = Pulley.State.Off;
                        intake.state = State.STOPPED;
                        holder.state = Servo.State.RESET;

                        telemetry.addData("Cycle", "Ball1 detected → stopping");
                        return false;
                    }

                    // If ran too long → stop
                    if (timer.milliseconds() > detectTimeout) {
                        pulley.state = Pulley.State.Off;
                        intake.state = State.STOPPED;
                        holder.state = Servo.State.RESET;

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
                    initialized = true;

                    pulley.state = Pulley.State.On;
                    intake.state = State.INTAKING;
                }

                if(timer.milliseconds()>=pulleyShootTime){
                    pulley.state = Pulley.State.Off;
                    intake.state = State.STOPPED;
                    holder.state = Servo.State.STOP;

                    return false;
                }
                return true;
            }
        };
    }

    public SequentialAction ShootThrough() {
        return new SequentialAction(
                StartShooter,
                WaitAction(speedUpTime),
                ReleaseBall,
                WaitAction(servoReleaseTime),
                Shoot(),
                CycleShoot(),
                StopShooter
        );
    }

    public SequentialAction AutoShoot() {
        return new SequentialAction(
                ReleaseBall,
                WaitAction(servoReleaseTime),
                Shoot(),
                CycleShoot(),
                StopShooter
        );
    }

    public SequentialAction ShootFirstBall() {
        return new SequentialAction(
                StartShooter,
                WaitAction(speedUpTime),
                ReleaseBall,
                WaitAction(servoReleaseTime),
                Shoot(),
                HoldBall
        );
    }

    public SequentialAction ShootBall() {
        return new SequentialAction(
                Cycle(),
                WaitAction(servoReleaseTime),
                Shoot(),
                HoldBall
        );
    }

    public SequentialAction ShootLastBall() {
        return new SequentialAction(
                Cycle(),
                WaitAction(servoReleaseTime),
                Shoot(),
                HoldBall,
                StopShooter
        );
    }

}
