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
    Servo holder;
    public ColorSensors outtake;
    public ColorSensors stop;

    Intake intake;
    Launcher launcher;

    Pulley pulley;

    //apriltag will dictate the motif number
    public int motif;

    //TODO() Need to make motor subsystem, find Servo values, and get a color sensor to check purple/green. Also make tests


    public void update() {
        aprilTag.update();
        holder.update();
        intake.update();
        launcher.update();
        pulley.update();
    }



    public Comp1Actions(HardwareMap hardwareMap) {
        aprilTag = new AprilTag(hardwareMap);
        holder = new Servo(hardwareMap, "port1");
        outtake = new ColorSensors(hardwareMap, "sensor1");
        launcher = new Launcher(hardwareMap);
        intake = new Intake(hardwareMap);
        pulley = new Pulley(hardwareMap);
        stop = new ColorSensors(hardwareMap, "sensor2");
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

    public Action OutakeColor = new Action() {

        final ElapsedTime timer = new ElapsedTime();
        boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                timer.reset();
                initialized = true;
            }

            return !outtake.checkForRecognition() && timer.milliseconds()<=timeoutMilliseconds;
        }
    };

    public Action StopColor = new Action() {

        final ElapsedTime timer = new ElapsedTime();
        boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                timer.reset();
                initialized = true;
            }

            return !stop.checkForRecognition() && timer.milliseconds()<=timeoutMilliseconds;
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
            holder.state = Servo.State.HOLD;

            return false;
        }
    };

    public SequentialAction Ball1 = new SequentialAction(StartIntake, OutakeColor, StopIntake);
    public SequentialAction Ball2 = new SequentialAction(StartIntake, StopColor, StopIntake);
    public SequentialAction Ball3 = new SequentialAction(StartIntake, new SleepAction(0.5), StopIntake);

    int shootTime = 500;
    int speedUpTime = 500;
    public double shootingInterval = 500;

    public Action Shoot = new Action() {
        final ElapsedTime timer = new ElapsedTime();
        boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(!initialized){
                timer.reset();
                initialized = true;
                launcher.setSpeed(0.8);
            }

            if(timer.milliseconds() >= speedUpTime && initialized){
                holder.state = Servo.State.LAUNCH;
            }
            if(timer.milliseconds() >= shootTime && initialized){
                holder.state = Servo.State.RESET;
                launcher.stop();
                return false;
            }
            else return true;


         }
    };
    public Action Cycle = new Action() {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            pulley.state = Pulley.State.On;

            if (outtake.checkForRecognition()){
                pulley.state = Pulley.State.Off;
                holder.state = Servo.State.HOLD;
                return false;
            }
            else return true;
        }
    };
}
