package org.firstinspires.ftc.teamcode.Competition_Code.Subsystems

import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.NormalizedColorSensor
import org.firstinspires.ftc.robotcore.external.JavaUtil
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import android.graphics.Color


class ColorSensors(hwMap: HardwareMap, name: String) {

    /**
     * A simple temporary variable that will hold the
     * converted RGB to HSV values
     * */
    val hsvValues = FloatArray(3)

    /**
     * The handle to our color sensor
     */
    private val colorSensor: ColorSensor =
        hwMap.get(ColorSensor::class.java, name)

    /**
     * This function checks to see if there is anything in
     * front of the color sensor
     */
    fun checkForRecognition(): Boolean {
        // Get the colors from the right color sensor
        (colorSensor as NormalizedColorSensor).gain = 2f
        val normalizedColors = (colorSensor as NormalizedColorSensor).normalizedColors

        // Convert the color into a packed 32-bit
        // Integer in the AARRGGBB format
        val color = normalizedColors.toColor()

        // Get the value of the color (aka the color itself)
        val value = JavaUtil.colorToValue(color)

        // Check if something is in front of the sensor
        // Checks:
        // - An object must be within 7 cm
        // - An object must have a color value
        //   of at least 0.05 (basically black)
        val inside =
            ((colorSensor as DistanceSensor).getDistance(DistanceUnit.CM) < 7) && value >= .05

        return inside
    }

    /**
     * Checks if the color green is in front of the sensor
     */
    fun isGreen(): Boolean {
        // Convert the RGB output to
        Color.RGBToHSV(
            (colorSensor.red() * 255) / 800,
            (colorSensor.green() * 255) / 800,
            (colorSensor.blue() * 255) / 800,
            hsvValues
        )

        val hue = hsvValues[0]

        return  hue in 100f..140f
    }

    /**
     * Checks if the color purple is in front of the sensor
     */
    fun isPurple(): Boolean {
        Color.RGBToHSV(
            (colorSensor.red() * 255) / 800,
            (colorSensor.green() * 255) / 800,
            (colorSensor.blue() * 255) / 800,
            hsvValues
        )

        val hue = hsvValues[0]

        return hue in 250f..290f
    }

}