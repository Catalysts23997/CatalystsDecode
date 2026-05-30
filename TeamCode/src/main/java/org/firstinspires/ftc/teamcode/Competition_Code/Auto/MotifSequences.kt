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
            robot.WaitAction(200.0),

            AutoPoints.EjectBlue.runToExact(),
            robot.EjectOne(),
            AutoPoints.LaunchBlue.runToExact(),
            robot.Shoot(),

            AutoPoints.PreGPPBlue.runToFast(),
            robot.StartIntake,
            AutoPoints.GPPBlue.runToExact(),
            robot.WaitAction(200.0),
            AutoPoints.GPPBackBlue.runToFast(),

            AutoPoints.EjectBlue.runToExact(),
            robot.EjectOne(),
            AutoPoints.LaunchBlue.runToExact(),
            robot.ShootSlow(),

            AutoPoints.PrePGPBlue.runToFast(),
            robot.StartIntake,
            AutoPoints.PGPBlue.runToExact(),
            robot.WaitAction(200.0),

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
            robot.WaitAction(200.0),
            AutoPoints.PGPBackBlue.runToFast(),

            AutoPoints.LaunchBlue.runToExact(),
            robot.ShootSlow(),

            AutoPoints.PrePPGBlue.runToFast(),
            robot.StartIntake,
            AutoPoints.PPGBlue.runToExact(),
            robot.WaitAction(200.0),


            AutoPoints.EjectBlue.runToExact(),
            robot.EjectTwo(),
            AutoPoints.LaunchBlue.runToExact(),
            robot.ShootFast(),

            AutoPoints.PreGPPBlue.runToFast(),
            robot.StartIntake,
            AutoPoints.GPPBlue.runToExact(),
            robot.WaitAction(200.0),

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
            robot.WaitAction(200.0),
            AutoPoints.GPPBackBlue.runToFast(),

            AutoPoints.EjectBlue.runToExact(),
            robot.EjectOne(),
            AutoPoints.LaunchBlue.runToExact(),

            robot.ShootSlow(),

            AutoPoints.PrePGPBlue.runToFast(),
            robot.StartIntake,
            AutoPoints.PGPBlue.runToExact(),
            robot.WaitAction(200.0),
            AutoPoints.PGPBackBlue.runToFast(),


            AutoPoints.LaunchBlue.runToExact(),
            robot.ShootSlow(),

            AutoPoints.PrePPGBlue.runToFast(),
            robot.StartIntake,
            AutoPoints.PPGBlue.runToExact(),
            robot.WaitAction(200.0),

            AutoPoints.LaunchBlue.runToExact(),
            robot.ShootSlow(),

            AutoPoints.EndBlue.runToExact()
        )
    }

    fun RPPG(): SequentialAction {
        return SequentialAction(
            AutoPoints.LaunchRed.runToExact(),
            robot.Shoot(),

            AutoPoints.PrePPGRed.runToFast(),
            robot.StartIntake,
            AutoPoints.PPGRed.runToExact(),
            robot.WaitAction(200.0),

            AutoPoints.EjectRed.runToExact(),
            robot.EjectOne(),
            AutoPoints.LaunchRed.runToExact(),
            robot.Shoot(),

            AutoPoints.PreGPPRed.runToFast(),
            robot.StartIntake,
            AutoPoints.GPPRed.runToExact(),
            robot.WaitAction(200.0),
            AutoPoints.GPPBackRed.runToFast(),

            AutoPoints.EjectRed.runToExact(),
            robot.EjectOne(),
            AutoPoints.LaunchRed.runToExact(),
            robot.ShootSlow(),

            AutoPoints.PrePGPRed.runToFast(),
            robot.StartIntake,
            AutoPoints.PGPRed.runToExact(),
            robot.WaitAction(200.0),

            AutoPoints.LaunchRed.runToExact(),
            robot.ShootSlow(),

            AutoPoints.EndRed.runToExact()
        )
    }
    fun RPGP(): SequentialAction {
        return SequentialAction(
            AutoPoints.LaunchRed.runToExact(),
            robot.Shoot(),

            AutoPoints.PrePGPRed.runToFast(),
            robot.StartIntake,
            AutoPoints.PGPRed.runToExact(),
            robot.WaitAction(200.0),
            AutoPoints.PGPBackRed.runToFast(),

            AutoPoints.LaunchRed.runToExact(),
            robot.ShootSlow(),

            AutoPoints.PrePPGRed.runToFast(),
            robot.StartIntake,
            AutoPoints.PPGRed.runToExact(),
            robot.WaitAction(200.0),

            AutoPoints.EjectRed.runToExact(),
            robot.EjectTwo(),
            AutoPoints.LaunchRed.runToExact(),
            robot.ShootFast(),

            AutoPoints.PreGPPRed.runToFast(),
            robot.StartIntake,
            AutoPoints.GPPRed.runToExact(),
            robot.WaitAction(200.0),

            AutoPoints.LaunchRed.runToExact(),
            robot.ShootSlow(),

            AutoPoints.EndRed.runToExact()
        )
    }
    fun RGPP(): SequentialAction {
        return SequentialAction(
            AutoPoints.LaunchRed.runToExact(),
            robot.Shoot(),

            AutoPoints.PreGPPRed.runToFast(),
            robot.StartIntake,
            AutoPoints.GPPRed.runToExact(),
            robot.WaitAction(200.0),
            AutoPoints.GPPBackRed.runToFast(),

            AutoPoints.EjectRed.runToExact(),
            robot.EjectOne(),
            AutoPoints.LaunchRed.runToExact(),

            robot.ShootSlow(),

            AutoPoints.PrePGPRed.runToFast(),
            robot.StartIntake,
            AutoPoints.PGPRed.runToExact(),
            robot.WaitAction(200.0),
            AutoPoints.PGPBackRed.runToFast(),


            AutoPoints.LaunchRed.runToExact(),
            robot.ShootSlow(),

            AutoPoints.PrePPGRed.runToFast(),
            robot.StartIntake,
            AutoPoints.PPGRed.runToExact(),
            robot.WaitAction(200.0),

            AutoPoints.LaunchRed.runToExact(),
            robot.ShootSlow(),

            AutoPoints.EndRed.runToExact()
        )
    }
}