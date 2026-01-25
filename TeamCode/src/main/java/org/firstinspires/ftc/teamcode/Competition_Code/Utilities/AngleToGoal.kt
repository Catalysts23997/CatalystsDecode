package org.firstinspires.ftc.teamcode.Competition_Code.Utilities

import org.firstinspires.ftc.teamcode.Competition_Code.AllianceColor
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Drivetrain
import kotlin.math.abs
import kotlin.math.atan

var basketX: Double = 72.0
const val basketY: Double = 72.0

var multiply = 1.0

fun goalAngle (currentX: Double, currentY: Double, alliance: AllianceColor): Double {
    if(alliance == AllianceColor.Blue){
        multiply = -1.0
        basketX = -72.0
    }
    else {
        multiply = 1.0
        basketX = 72.0
    }

    val angle = multiply*(Math.PI + atan(abs(basketX - currentX) / abs(basketY - currentY)))


    return angle
}