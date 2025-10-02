package org.firstinspires.ftc.teamcode.Competition_Code.Auto.OpModes;

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.Competition_Code.Actions.Comp1Actions

import org.firstinspires.ftc.teamcode.Competition_Code.Auto.AutoPoints
import org.firstinspires.ftc.teamcode.Competition_Code.Auto.RunToExactForever
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Drivetrain
import org.firstinspires.ftc.teamcode.Competition_Code.PinpointLocalizer.Localizer
import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.Poses

@Autonomous(name = "BlueAuto", group = "Auto")
class BlueAuto : LinearOpMode() {

    companion object{
        var rT = Poses(-41.0,60.0,-Math.PI/2)
    }

    override fun runOpMode() {
        telemetry = FtcDashboard.getInstance().telemetry
        rT = Poses(-41.0,60.0,-Math.PI/2)

        val localizer = Localizer(hardwareMap, rT)
        val drive = Drivetrain(hardwareMap)
        val robot = Comp1Actions(hardwareMap)

        localizer.update()
        robot.update()

        waitForStart()

        runBlocking(
            ParallelAction(
                {
                    localizer.update()
                    RunToExactForever(rT)
                    telemetry.addData("hello", rT)
                    telemetry.addData("df", Localizer.pose.heading)
                    telemetry.addData("x", Localizer.pose.x)
                    telemetry.addData("y", Localizer.pose.y)
                    telemetry.update()
                    robot.update()
                    true
                },
                SequentialAction(
                    AutoPoints.AprilTagBlue.runToExact,
                    robot.CheckMotif,
                    AutoPoints.LaunchBlue.runToExact,
                    robot.Shoot,
                    SleepAction(robot.shootingInterval/1000),
                    robot.Shoot,
                    SleepAction(robot.shootingInterval/1000),
                    robot.Shoot,

                    when (robot.motif) {
                        1 -> {
                            SequentialAction(
                                AutoPoints.PreIntakeGPP.runToExact,
                                robot.StartIntake,
                                ParallelAction(
                                    AutoPoints.GPPIntake1.runToExact,
                                    robot.CheckColor
                                ),
                                ParallelAction(
                                    AutoPoints.GPPIntake2.runToExact,
                                    robot.CheckColor
                                ),
                                ParallelAction(
                                    AutoPoints.GPPIntake3.runToExact,
                                    robot.CheckColor
                                ),
                                robot.StopIntake,
                                AutoPoints.PreIntakePGP.runToExact
                            )
                        }
                        2 -> {
                            SequentialAction(
                                AutoPoints.PreIntakePGP.runToExact,
                                robot.StartIntake,
                                ParallelAction(
                                    AutoPoints.PGPIntake1.runToExact,
                                    robot.CheckColor
                                ),
                                ParallelAction(
                                    AutoPoints.PGPIntake2.runToExact,
                                    robot.CheckColor
                                ),
                                ParallelAction(
                                    AutoPoints.PGPIntake3.runToExact,
                                    robot.CheckColor
                                ),
                                robot.StopIntake,
                                AutoPoints.PreIntakePPG.runToExact
                            )
                        }
                        else -> {
                            SequentialAction(
                                AutoPoints.PreIntakePPG.runToExact,
                                robot.StartIntake,
                                ParallelAction(
                                    AutoPoints.PPGIntake1.runToExact,
                                    robot.CheckColor
                                ),
                                ParallelAction(
                                    AutoPoints.PPGIntake2.runToExact,
                                    robot.CheckColor
                                ),
                                ParallelAction(
                                    AutoPoints.PPGIntake3.runToExact,
                                    robot.CheckColor
                                ),
                                robot.StopIntake
                            )
                        }
                    },
                    AutoPoints.LaunchBlue.runToExact,
                    robot.Shoot,
                    SleepAction(robot.shootingInterval/1000),
                    robot.Shoot,
                    SleepAction(robot.shootingInterval/1000),
                    robot.Shoot,
                    AutoPoints.EndBlue.runToExact
                )
            )
        )

    }
}
