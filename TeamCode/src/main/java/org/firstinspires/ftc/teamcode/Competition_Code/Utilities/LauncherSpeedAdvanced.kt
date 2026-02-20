package org.firstinspires.ftc.teamcode.Competition_Code.Utilities

import org.firstinspires.ftc.teamcode.Competition_Code.AllianceColor
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sin
import kotlin.math.sqrt

val inpersec = 133.0
val rpm = 2500.0
val rpmToips = inpersec/rpm


fun launcherSpeedAdjusted (xVelocity:Double, yVelocity: Double, currentX: Double, currentY: Double, alliance: AllianceColor): Double{
    val baseSpeed = launcherSpeed(currentX,currentY,alliance)*rpmToips
    val heading = goalAngle(currentX,currentY,alliance)
    val adjustedSpeed = sqrt((baseSpeed* sin(heading)-xVelocity).pow(2)+(baseSpeed* cos(heading)-yVelocity).pow(2))/rpmToips
    return adjustedSpeed
}