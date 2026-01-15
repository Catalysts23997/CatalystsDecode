package org.firstinspires.ftc.teamcode.Competition_Code.Utilities

import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Drivetrain
import kotlin.math.abs
import kotlin.math.atan

var basketx: Double = 72.0
val baskety: Double = 72.0

var multiply = 1.0

fun goalAngle (currentx: Double, currenty: Double, alliance: Drivetrain.Alliance): Double {
    if(alliance == Drivetrain.Alliance.Blue){
        multiply = -1.0
        basketx = -72.0
    }

    var angle = multiply * atan(abs(basketx - currentx) / abs(baskety - currenty))


    return angle
}