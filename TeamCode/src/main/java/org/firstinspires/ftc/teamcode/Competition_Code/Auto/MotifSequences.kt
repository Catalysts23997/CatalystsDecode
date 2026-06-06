package org.firstinspires.ftc.teamcode.Competition_Code.Auto

import com.acmerobotics.roadrunner.SequentialAction
import org.firstinspires.ftc.teamcode.Competition_Code.Actions.InterleagueActions

class MotifSequences(val robot: InterleagueActions) {

    fun PPG(): SequentialAction {
        return SequentialAction(
            AutoPoints.LaunchBlue.runToExact(),
            robot.Shoot(),

            AutoPoints.PrePPGBlue.runToFast(),
            robot.StartIntake,
            AutoPoints.PPGBlue.runToExact(),

            AutoPoints.EjectBlue.runToExact(),
            robot.EjectOne(),
            AutoPoints.LaunchBlue.runToExact(),
            robot.Shoot(),

            AutoPoints.PreGPPBlue.runToFast(),
            robot.StartIntake,
            AutoPoints.GPPBlue.runToExact(),
            AutoPoints.GPPBackBlue.runToFast(),

            AutoPoints.EjectBlue.runToExact(),
            robot.EjectOne(),
            AutoPoints.LaunchBlue.runToExact(),
            robot.ShootSlow(),

            AutoPoints.PrePGPBlue.runToFast(),
            robot.StartIntake,
            AutoPoints.PGPBlue.runToExact(),

            AutoPoints.LaunchBlue.runToExact(),
            robot.ShootSlow(),

            AutoPoints.EndBlue.runToExact()
        )
    }
    fun PGP(): SequentialAction {
        return SequentialAction(
            AutoPoints.LaunchBlue.runToExact(),
            robot.Shoot(),

            AutoPoints.PrePGPBlue.runToFast(),
            robot.StartIntake,
            AutoPoints.PGPBlue.runToExact(),
            AutoPoints.PGPBackBlue.runToFast(),

            AutoPoints.LaunchBlue.runToExact(),
            robot.ShootSlow(),

            AutoPoints.PrePPGBlue.runToFast(),
            robot.StartIntake,
            AutoPoints.PPGBlue.runToExact(),

            AutoPoints.EjectBlue.runToExact(),
            robot.EjectTwo(),
            AutoPoints.LaunchBlue.runToExact(),
            robot.ShootFast(),

            AutoPoints.PreGPPBlue.runToFast(),
            robot.StartIntake,
            AutoPoints.GPPBlue.runToExact(),

            AutoPoints.LaunchBlue.runToExact(),
            robot.ShootSlow(),

            AutoPoints.EndBlue.runToExact()
        )
    }
    fun GPP(): SequentialAction {
        return SequentialAction(
            AutoPoints.LaunchBlue.runToExact(),
            robot.Shoot(),

            AutoPoints.PreGPPBlue.runToFast(),
            robot.StartIntake,
            AutoPoints.GPPBlue.runToExact(),
            AutoPoints.GPPBackBlue.runToFast(),

            AutoPoints.EjectBlue.runToExact(),
            robot.EjectOne(),
            AutoPoints.LaunchBlue.runToExact(),

            robot.ShootSlow(),

            AutoPoints.PrePGPBlue.runToFast(),
            robot.StartIntake,
            AutoPoints.PGPBlue.runToExact(),
            AutoPoints.PGPBackBlue.runToFast(),


            AutoPoints.LaunchBlue.runToExact(),
            robot.ShootSlow(),

            AutoPoints.PrePPGBlue.runToFast(),
            robot.StartIntake,
            AutoPoints.PPGBlue.runToExact(),

            AutoPoints.LaunchBlue.runToExact(),
            robot.ShootSlow(),

            AutoPoints.EndBlue.runToExact()
        )
    }
}