package org.firstinspires.ftc.teamcode.Competition_Code.Auto

import net.mccoder.ftvision.processors.LocationWithVelocity
import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.Poses

object AutoGlobals {

    var FarAuto: Boolean = false
    var AutonomousRan: Boolean = false
    var locationOfRobot: Poses? = null
    var driveSpeed = 1.0
    var targetRobotPositon: Poses = Poses(0.0,0.0,0.0)

    var detectedBall: LocationWithVelocity? = null
    var detectedBallPose: Poses? = null
}