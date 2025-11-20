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
import org.firstinspires.ftc.teamcode.Competition_Code.Auto.OpModes.BlueAuto.Companion.endPos
import org.firstinspires.ftc.teamcode.Competition_Code.Auto.RunToExactForever
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Drivetrain
import org.firstinspires.ftc.teamcode.Competition_Code.PinpointLocalizer.Localizer
import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.Poses

@Autonomous(name = "BlueAutoTest", group = "Auto")
class BlueAutoTests : LinearOpMode() {


    override fun runOpMode() {
        telemetry = FtcDashboard.getInstance().telemetry
        rT = Poses(-39.0,63.0,0.0)

        val localizer = Localizer(hardwareMap, rT)
        val drive = Drivetrain(hardwareMap)
        val motif = 3

        localizer.update()

        waitForStart()

        runBlocking(
            ParallelAction(
                {
                    localizer.update()
                    RunToExactForever(rT, 1.0)
                    endPos = Localizer.pose
                    telemetry.addData("hello", rT)
                    telemetry.addData("df", Localizer.pose.heading)
                    telemetry.addData("x", Localizer.pose.x)
                    telemetry.addData("y", Localizer.pose.y)
                    telemetry.update()
                    true
                },
                SequentialAction(
                    AutoPoints.AprilTagBlue.runToExact,
                    AutoPoints.LaunchBlue.runToExact,
                    SleepAction(1.0),

                    when (motif) {
                        1 -> {
                            SequentialAction(
                                AutoPoints.PreIntakeGPP.runToExact,
                                AutoPoints.GPPIntake1.runToExact,
                                AutoPoints.GPPIntake2.runToExact,
                                AutoPoints.GPPIntake3.runToExact,
                                AutoPoints.GPPMidPointRed.runToExact
                            )
                        }
                        2 -> {
                            SequentialAction(
                                AutoPoints.PreIntakePGP.runToExact,
                                AutoPoints.PGPIntake1.runToExact,
                                AutoPoints.PGPIntake2.runToExact,
                                AutoPoints.PGPIntake3.runToExact,
                                AutoPoints.PGPMidPointRed.runToExact
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
                    AutoPoints.LaunchBlue.runToExact,
                    AutoPoints.EndBlue.runToExact
                )
            )
        )

    }
}
