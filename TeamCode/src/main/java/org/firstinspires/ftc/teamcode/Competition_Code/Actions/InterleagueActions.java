package org.firstinspires.ftc.teamcode.Competition_Code.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Competition_Code.Auto.AutoGlobals;
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.AprilTag;
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.ColorSensors;
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Pulley;
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Servo;
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Intake.State;
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.SingleLauncher;

public class InterleagueActions {
    AprilTag aprilTag;


    public int motif;

    public ColorSensors ball1;
    public ColorSensors ball2;

    public Servo holder;

    public Intake intake;
    Pulley pulley;

    public SingleLauncher launcher;

    Telemetry telemetry;

    public void update() {
        if (!AutoGlobals.INSTANCE.getAutonomousRan()){
            aprilTag.update();
        }
        ;

        intake.update();
        pulley.update();

        holder.update();

        launcher.update();

        telemetry.addData("Ball1 Is Green?", ball1.isGreen());
        telemetry.addData("Ball1 Is Purple?", ball1.isPurple());
        telemetry.addData("Ball1 Hue?", ball1.getHue());
        telemetry.addData("Ball2 Is Green?", ball2.isGreen());
        telemetry.addData("Ball2 Is Purple?", ball2.isPurple());
        telemetry.addData("Ball2 Hue?", ball2.getHue());

        telemetry.addData("Intake State", intake.state);
        telemetry.addData("Pulley State", pulley.state);

        telemetry.addData("Holder State", holder.state);

        telemetry.addData("Left Launcher Speed", launcher.getRpm());


    }

    public InterleagueActions(HardwareMap hardwareMap, Telemetry telemetry) {
        if (!AutoGlobals.INSTANCE.getAutonomousRan()){
            aprilTag = new AprilTag(hardwareMap);
        }

        ball1 = new ColorSensors(hardwareMap, "ball1");
        ball2 = new ColorSensors(hardwareMap, "ball2");

        intake = new Intake(hardwareMap);
        pulley = new Pulley(hardwareMap);

        holder = new Servo(hardwareMap,"holder");

        launcher = new SingleLauncher(hardwareMap);

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
    double cameraTimeout = 1000;

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
            launcher.reverse();

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
            holder.state = Servo.State.STOP1;

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

    //shooting stuff
    double speedUpTime = 1300;      // time for flywheel to reach speed
    double servoReleaseTime = 800;  // ms for servo release
    double pulleyShootTime = 1600;  // ms for pulley to shoot 3 balls

    public Action StartShooter = new Action() {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            launcher.start();

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

                    pulley.state = Pulley.State.Slow;
                    intake.state = State.INTAKING;
                }

                if(timer.milliseconds()>=pulleyShootTime){
                    pulley.state = Pulley.State.Off;
                    intake.state = State.STOPPED;
                    holder.state = Servo.State.STOP1;

                    return false;
                }
                return true;
            }
        };
    }

    double toleranceRPM = 100;

    public Action WaitForLauncher() {
        return new Action() {

            private final ElapsedTime timer = new ElapsedTime();

            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                if (!initialized) {
                    timer.reset();
                    initialized = true;
                }

                boolean atSpeed = launcher.atTargetRPM(launcher.getGoalRPM(), toleranceRPM);


                telemetry.addData("Launcher At Speed", atSpeed);
                telemetry.addData("Launcher Timer (ms)", timer.milliseconds());

                // Keep running while:
                //  - NOT at speed
                //  - AND timeout not exceeded
                return !(atSpeed || timer.milliseconds() > (speedUpTime -  servoReleaseTime));
            }
        };
    }


    public SequentialAction Shoot() {
        return new SequentialAction(
                StartShooter,
                StopIntake,
                ReleaseBall,
                WaitAction(servoReleaseTime),
                WaitForLauncher(),
                CycleShoot()
        );
    }



}