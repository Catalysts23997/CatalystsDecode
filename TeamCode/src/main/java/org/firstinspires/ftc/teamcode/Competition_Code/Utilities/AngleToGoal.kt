package org.firstinspires.ftc.teamcode.Competition_Code.Utilities

import org.firstinspires.ftc.teamcode.Competition_Code.AllianceColor
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Drivetrain
import kotlin.math.abs
import kotlin.math.atan
import kotlin.math.atan2

fun goalAngle(
    currentX: Double,
    currentY: Double,
    alliance: AllianceColor
): Double {

    val basketX = if (alliance == AllianceColor.Blue) -67.0 else 67.0
    val basketY = 67.0

    val dx = basketX - currentX
    val dy = basketY - currentY

    return atan2(dx, dy)
}
