package org.firstinspires.ftc.teamcode.Competition_Code.Auto.OpModes;

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.SequentialAction
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
                    robot.Sleep(500.0),
                    robot.Shoot,
                    robot.Sleep(500.0),
                    robot.Shoot,

                    when (robot.motif) {
                        1 -> {
                            SequentialAction(
                                AutoPoints.PreIntakeGPP.runToExact,
                                ParallelAction(
                                    robot.Intake,
                                    AutoPoints.GPPIntake1.runToExact
                                ),
                                ParallelAction(
                                    robot.Intake,
                                    AutoPoints.GPPIntake2.runToExact
                                ),
                                ParallelAction(
                                    robot.Intake,
                                    AutoPoints.GPPIntake3.runToExact
                                ),
                            )
                        }
                        2 -> {
                            SequentialAction(
                                AutoPoints.PreIntakePGP.runToExact,
                                ParallelAction(
                                    robot.Intake,
                                    AutoPoints.PGPIntake1.runToExact
                                ),
                                ParallelAction(
                                    robot.Intake,
                                    AutoPoints.PGPIntake2.runToExact
                                ),
                                ParallelAction(
                                    robot.Intake,
                                    AutoPoints.PGPIntake3.runToExact
                                ),
                            )
                        }
                        else -> {
                            SequentialAction(
                                AutoPoints.PreIntakePPG.runToExact,
                                ParallelAction(
                                    robot.Intake,
                                    AutoPoints.PPGIntake1.runToExact
                                ),
                                ParallelAction(
                                    robot.Intake,
                                    AutoPoints.PPGIntake2.runToExact
                                ),
                                ParallelAction(
                                    robot.Intake,
                                    AutoPoints.PPGIntake3.runToExact
                                ),
                            )
                        }
                    },
                    AutoPoints.LaunchBlue.runToExact,
                    robot.Shoot,
                    robot.Sleep(500.0),
                    robot.Shoot,
                    robot.Sleep(500.0),
                    robot.Shoot,
                    AutoPoints.EndBlue.runToExact
                )
            )
        )

    }
}
