package org.firstinspires.ftc.teamcode.Competition_Code.Utilities

import org.firstinspires.ftc.teamcode.Competition_Code.AllianceColor
import kotlin.math.PI
import kotlin.math.atan
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin

fun goalAngleAdjusted(xVelocity:Double, yVelocity: Double, currentX: Double, currentY: Double, alliance: AllianceColor): Double {
    val baseSpeed = launcherSpeed(currentX,currentY,alliance)*rpmToips
    val heading = goalAngle(currentX,currentY,alliance)
    val adjustedHeading = atan2((baseSpeed* sin(heading)-xVelocity),(baseSpeed* cos(heading) -yVelocity))
    return  adjustedHeading + PI
}