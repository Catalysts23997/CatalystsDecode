package org.firstinspires.ftc.teamcode.Competition_Code.Utilities

import org.firstinspires.ftc.teamcode.Competition_Code.AllianceColor
import kotlin.math.pow
import kotlin.math.sqrt



fun launcherSpeed(currentX: Double, currentY: Double, alliance: AllianceColor): Double {
    val basketX = if(alliance == AllianceColor.Blue) -72.0
    else 72.0


    val distance = sqrt((basketX-currentX).pow(2)+(72.0-currentY).pow(2))

    val rpm = 11*distance+1720

    return rpm
}