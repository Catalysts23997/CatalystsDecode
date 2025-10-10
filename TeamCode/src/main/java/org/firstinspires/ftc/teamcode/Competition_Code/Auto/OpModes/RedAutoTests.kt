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

@Autonomous(name = "RedAutoTests", group = "Auto")
class RedAutoTests : LinearOpMode() {

    companion object{
        var rT = Poses(-41.0,60.0,-Math.PI/2)
    }

    override fun runOpMode() {
        telemetry = FtcDashboard.getInstance().telemetry
        rT = Poses(-41.0,60.0,-Math.PI/2)

        val localizer = Localizer(hardwareMap, rT)
        val drive = Drivetrain(hardwareMap)
        val motif = 1

        localizer.update()

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
                    true
                },
                SequentialAction(
                    AutoPoints.AprilTagRed.runToExact,
                    AutoPoints.LaunchRed.runToExact,

                    when (motif) {
                        1 -> {
                            SequentialAction(
                                AutoPoints.PreIntakeGPP.runToExact,
                                AutoPoints.GPPIntake1.runToExact,
                                AutoPoints.GPPIntake2.runToExact,
                                AutoPoints.GPPIntake3.runToExact,
                                AutoPoints.PreIntakePGP.runToExact
                            )
                        }
                        2 -> {
                            SequentialAction(
                                AutoPoints.PreIntakePGP.runToExact,
                                AutoPoints.PGPIntake1.runToExact,
                                AutoPoints.PGPIntake2.runToExact,
                                AutoPoints.PGPIntake3.runToExact,
                                AutoPoints.PreIntakePPG.runToExact
                            )
                        }
                        else -> {
                            SequentialAction(
                                AutoPoints.PreIntakePPG.runToExact,
                                AutoPoints.PPGIntake1.runToExact,
                                AutoPoints.PPGIntake2.runToExact,
                                AutoPoints.PPGIntake3.runToExact,
                            )
                        }
                    },
                    AutoPoints.LaunchRed.runToExact,
                    AutoPoints.EndRed.runToExact
                )
            )
        )

    }
}
