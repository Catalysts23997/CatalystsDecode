package org.firstinspires.ftc.teamcode.Competition_Code.Auto

import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.Poses

object AutoGlobals {

    var AutonomousRan: Boolean = false
    var locationOfRobot: Poses? = null
    var driveSpeed = 1.0
    var targetRobotPositon: Poses = Poses(0.0,0.0,0.0)
    val slow = 0.75
}