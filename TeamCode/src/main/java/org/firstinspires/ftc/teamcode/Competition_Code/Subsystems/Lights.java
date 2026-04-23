package org.firstinspires.ftc.teamcode.Competition_Code.Subsystems;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
public class Lights {
    Servo blinkin;
    Servo blinkin2;
    public Color color = Color.off;
    double count;
    double change = 0.01;

    public Lights(HardwareMap hardwareMap) {
        blinkin = hardwareMap.get(Servo.class, "blinkin");
        blinkin2 = hardwareMap.get(Servo.class, "blinkin2");

    }

    public void update() {
        if(color != Color.rainbow){
            count = 0.28;
            change = 0.01;
            blinkin.setPosition(color.value);
            blinkin2.setPosition(color.value);
        }
        else{
            blinkin.setPosition(count);
            blinkin2.setPosition(count);

            count += change;
            if(count > 0.72 || count < 0.28){
                change *= -1;
            }
        }

    }
    public enum Color {
        red(0.28),
        blue(0.62),
        green(0.5),
        yellow(0.388),
        white(1.0),
        purple(0.72),
        off(0.0),
        rainbow(10000);
        public final double value;
        Color(double value) {this.value = value;}
    }
}
