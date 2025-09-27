package org.firstinspires.ftc.teamcode.Competition_Code.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.AprilTag;
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.ColorSensors;
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Servo;
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Intake.State;

public class Comp1Actions {
    AprilTag aprilTag;
    Servo holder;
    ColorSensors outtake;

    Intake intake;
    Launcher launcher;

    //apriltag will dictate the motif number
    public int motif;

    //TODO() Need to make motor subsystem, find Servo values, and get a color sensor to check purple/green. Also make tests


    public void update() {
        aprilTag.update();
        holder.update();
        intake.update();
        launcher.update();
    }



    public Comp1Actions(HardwareMap hardwareMap) {
        aprilTag = new AprilTag(hardwareMap);
        holder = new Servo(hardwareMap, "port1");
        outtake = new ColorSensors(hardwareMap, "sensor1");
        launcher = new Launcher(hardwareMap);
        intake = new Intake(hardwareMap);
    }

    public Action CheckMotif = new Action() {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            motif = aprilTag.getMotif();
            return motif == 0;
        }
    };


        public Action Intake = new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                intake.state = State.INTAKING;
                holder.state = Servo.State.HOLD;

                if(outtake.checkForRecognition()){
                    holder.state = Servo.State.LAUNCH;
                    intake.state = State.STOPPED;
                    return false;
                }
                else return true;


            }
        };

        int shootTime = 500;

        public double shootingInterval = 500;

        public Action Shoot = new Action() {
            final ElapsedTime timer = new ElapsedTime();
            boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(!initialized){
                    timer.reset();
                    initialized = true;
                    holder.state = Servo.State.LAUNCH;
                    launcher.setSpeed(1.0);
                }

                if(timer.milliseconds() >= shootTime && initialized){
                    holder.state = Servo.State.HOLD;
                    launcher.stop();
                    return false;
                }
                else return true;


            }
        };



    public Action Sleep(double ms) {
        return new Action() {
            final ElapsedTime timer = new ElapsedTime();
            boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    timer.reset();
                    initialized = true;
                }

                return timer.seconds() < ms;
            }
        };
    }
}
