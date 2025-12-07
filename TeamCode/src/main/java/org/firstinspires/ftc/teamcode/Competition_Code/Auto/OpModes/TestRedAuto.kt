package org.firstinspires.ftc.teamcode.Competition_Code.Auto.OpModes;

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

import org.firstinspires.ftc.teamcode.Competition_Code.Auto.AutoPoints
import org.firstinspires.ftc.teamcode.Competition_Code.Auto.OpModes.BlueAuto6.Companion.rT
import org.firstinspires.ftc.teamcode.Competition_Code.Auto.RunToExactForever
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Drivetrain
import org.firstinspires.ftc.teamcode.Competition_Code.PinpointLocalizer.Localizer

@Disabled
@Autonomous(name = "TestRedAuto", group = "Auto")
class TestRedAuto : LinearOpMode() {


    override fun runOpMode() {
        telemetry = FtcDashboard.getInstance().telemetry
        rT = AutoPoints.StartRed.pose

        val localizer = Localizer(hardwareMap, rT)
        val drive = Drivetrain(hardwareMap)
        val motif = 3

        localizer.update()

        waitForStart()

        runBlocking(
            ParallelAction(
                {
                    localizer.update()
                    RunToExactForever(rT,1.0)
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
                                AutoPoints.GPPIntakeRed.runToExact,
                                AutoPoints.GPPMidPointRed.runToExact
                            )
                        }
                        2 -> {
                            SequentialAction(
                                AutoPoints.PreIntakePGPRed.runToExact,
                                AutoPoints.PGPIntakeRed.runToExact,
                                AutoPoints.PGPMidPointRed.runToExact
                            )
                        }
                        else -> {
                            SequentialAction(
                                AutoPoints.PreIntakePPGRed.runToExact,
                                AutoPoints.PPGIntakeRed.runToExact,
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
