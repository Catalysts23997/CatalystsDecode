package org.firstinspires.ftc.teamcode.Competition_Code.Subsystems;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
public class Lights {
    Servo blinkin;
    public Color color = Color.off;

    public Lights(HardwareMap hardwareMap) {
        blinkin = hardwareMap.get(Servo.class, "blinkin");
    }

    public void update() {
        blinkin.setPosition(color.value);
    }
    public enum Color {
        red(0.61),
        blue(0.77),
        green(0.77),
        white(0.11),
        rainbow(0.99),
        purple(0.45),
        off(1);
        public final double value;
        Color(double value) {this.value = value;}
    }
}
