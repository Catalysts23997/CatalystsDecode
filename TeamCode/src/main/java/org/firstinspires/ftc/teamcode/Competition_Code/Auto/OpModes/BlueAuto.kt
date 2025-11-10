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
        var rT = Poses(-39.0,63.0,Math.PI/2)
    }

    override fun runOpMode() {
        telemetry = FtcDashboard.getInstance().telemetry
        rT = Poses(-39.0,63.0,Math.PI/2)

        val localizer = Localizer(hardwareMap, rT)
        val drive = Drivetrain(hardwareMap)
        val robot = Comp1Actions(hardwareMap)

        localizer.update()
        robot.HoldBall
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
                    robot.ShootBalls,
                    when (robot.motif) {
                        1 -> {
                            SequentialAction(
                                AutoPoints.PreIntakeGPP.runToExact,
                                ParallelAction(
                                    AutoPoints.GPPIntake1.runToExact,
                                    robot.Ball1
                                ),
                                ParallelAction(
                                    AutoPoints.GPPIntake2.runToExact,
                                    robot.Ball2
                                ),
                                ParallelAction(
                                    AutoPoints.GPPIntake3.runToExact,
                                    robot.Ball3
                                ),
                                AutoPoints.GPPMidPoint.runToExact
                            )
                        }
                        2 -> {
                            SequentialAction(
                                AutoPoints.PreIntakePGP.runToExact,
                                ParallelAction(
                                    AutoPoints.PGPIntake1.runToExact,
                                    robot.Ball1
                                ),
                                ParallelAction(
                                    AutoPoints.PGPIntake2.runToExact,
                                    robot.Ball2
                                ),
                                ParallelAction(
                                    AutoPoints.PGPIntake3.runToExact,
                                    robot.Ball3
                                ),
                                AutoPoints.PGPMidPoint.runToExact
                            )
                        }
                        else -> {
                            SequentialAction(
                                AutoPoints.PreIntakePPG.runToExact,
                                ParallelAction(
                                    AutoPoints.PPGIntake1.runToExact,
                                    robot.Ball1
                                ),
                                ParallelAction(
                                    AutoPoints.PPGIntake2.runToExact,
                                    robot.Ball2
                                ),
                                ParallelAction(
                                    AutoPoints.PPGIntake3.runToExact,
                                    robot.Ball3
                                )
                            )
                        }
                    },
                    AutoPoints.LaunchBlue.runToExact,
                    robot.ShootBalls,
                    AutoPoints.EndBlue.runToExact
                )
            )
        )

    }
}
