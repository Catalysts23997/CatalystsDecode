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
import org.firstinspires.ftc.teamcode.Competition_Code.Auto.OpModes.BlueAuto.Companion.rT
import org.firstinspires.ftc.teamcode.Competition_Code.Auto.RunToExactForever
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Drivetrain
import org.firstinspires.ftc.teamcode.Competition_Code.PinpointLocalizer.Localizer
import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.Poses

@Autonomous(name = "RedAutoTests", group = "Auto")
class RedAutoTests : LinearOpMode() {


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
                                AutoPoints.PreIntakeGPPRed.runToExact,
                                AutoPoints.GPPIntake1Red.runToExact,
                                AutoPoints.GPPIntake2Red.runToExact,
                                AutoPoints.GPPIntake3Red.runToExact,
                                AutoPoints.GPPMidPointRed.runToExact
                            )
                        }
                        2 -> {
                            SequentialAction(
                                AutoPoints.PreIntakePGPRed.runToExact,
                                AutoPoints.PGPIntake1Red.runToExact,
                                AutoPoints.PGPIntake2Red.runToExact,
                                AutoPoints.PGPIntake3Red.runToExact,
                                AutoPoints.PGPMidPointRed.runToExact
                            )
                        }
                        else -> {
                            SequentialAction(
                                AutoPoints.PreIntakePPGRed.runToExact,
                                AutoPoints.PPGIntake1Red.runToExact,
                                AutoPoints.PPGIntake2Red.runToExact,
                                AutoPoints.PPGIntake3Red.runToExact,
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
