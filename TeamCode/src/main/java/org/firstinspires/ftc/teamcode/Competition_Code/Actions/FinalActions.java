package org.firstinspires.ftc.teamcode.Competition_Code.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.AprilTag;
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.ColorSensors;
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Servo;

public class FinalActions {
    AprilTag aprilTag;
    ColorSensors sorterColor;
    Servo sorter;
    Servo purpleLaunch;
    Servo greenLaunch;

    // Motor launch;
    // Motor intake;

    //apriltag will dictate the motif number
    public int motif;

    //TODO() Need to make motor subsystem, find Servo values, and get a color sensor to check purple/green. Also make tests


    public void update() {
        aprilTag.update();
        sorter.update();
        //intake.update();
        //launch.update():
        greenLaunch.update();
        purpleLaunch.update();
    }



    public FinalActions(HardwareMap hardwareMap) {
        aprilTag = new AprilTag(hardwareMap);
        sorterColor = new ColorSensors(hardwareMap, "something");
        sorter = new Servo(hardwareMap, "port1");
        purpleLaunch = new Servo(hardwareMap, "port2");
        greenLaunch = new Servo(hardwareMap, "port3");
        // launch = new Motor(hardwareMap);
        // intake = new Motor(hardwareMap);
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
            sorter.state = Servo.State.HOLD;
            //intake.state = Motor.State.Intake
            if(sorterColor.checkForRecognition()){
                return false;
                //intake.state = Motor.State.Stop

            }
            else return true;

        }
    };

    public Action Sort = new Action() {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            /*if(sorterColor.color == purple){
                sorter.state = Servo.State.PURPLE;

                return false;
            }
            else if (sorterColor.color == green) {
                sorter.state = Servo.State.GREEN;

                return false;
            }
            else return true;

             */
            return false;

        }
    };

    public SequentialAction SortBall = new SequentialAction(Intake, Sort);

    public Action ShootGreen = new Action() {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            //launch.state = Motor.State.Launch
//            greenLaunch.state= Servo.State.LAUNCHG;
//            //wait for ball to go out
//            greenLaunch.state= Servo.State.HOLDG;
            //launch.state = Motor.State.Stop
            return false;
        }
    };

    public Action ShootPurple = new Action() {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            //launch.state = Motor.State.Launch
//            purpleLaunch.state= Servo.State.LAUNCHP;
//            //wait for ball to go out
//            purpleLaunch.state= Servo.State.HOLDP;
            //launch.state = Motor.State.Stop
            return false;
        }
    };

    public SequentialAction ShootMotif1 = new SequentialAction(ShootGreen, ShootPurple, ShootPurple);

    public SequentialAction ShootMotif2 = new SequentialAction(ShootPurple, ShootGreen, ShootPurple);

    public SequentialAction ShootMotif3 = new SequentialAction(ShootPurple, ShootPurple, ShootGreen);

    public Action Endgame = new Action() {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            //stuff
            return false;
        }
    };
}
