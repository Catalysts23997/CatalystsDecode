package org.firstinspires.ftc.teamcode.Competition_Code.Auto.OpModes;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.Competition_Code.Actions.Comp1Actions
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.teamcode.Competition_Code.Auto.AutoGlobals

import org.firstinspires.ftc.teamcode.Competition_Code.Auto.AutoPoints
import org.firstinspires.ftc.teamcode.Competition_Code.Auto.RunToExactForever
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Drivetrain
import org.firstinspires.ftc.teamcode.Competition_Code.PinpointLocalizer.Localizer
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Servo
import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.Poses

@Disabled
@Autonomous(name = "BlueAuto6", group = "Auto")
class BlueAuto6 : LinearOpMode() {


    override fun runOpMode() {
        AutoGlobals.targetRobotPositon = AutoPoints.StartBlue.pose

        val localizer = Localizer(hardwareMap, AutoGlobals.targetRobotPositon)
        val drive = Drivetrain(hardwareMap)
        val robot = Comp1Actions(hardwareMap, telemetry)

        sleep(100)
        localizer.update()
        robot.holder.state = Servo.State.STOP1
        robot.update()

        waitForStart()

        AutoGlobals.AutonomousRan = true

        localizer.update()
        localizer.transferToTele()

        runBlocking(
            ParallelAction(
                object : Action {
                    override fun run(p: TelemetryPacket): Boolean {
                        if (isStopRequested) {
                            stop()
                        }

                        localizer.update()
                        RunToExactForever(AutoGlobals.targetRobotPositon)
                        AutoGlobals.locationOfRobot = Poses(Localizer.pose.x, Localizer.pose.y, Localizer.pose.heading)
                        telemetry.addData("goalPos", AutoGlobals.targetRobotPositon)
                        telemetry.addData("heading", Localizer.pose.heading)
                        telemetry.addData("x", Localizer.pose.x)
                        telemetry.addData("y", Localizer.pose.y)
                        telemetry.update()
                        robot.update()
                        return true // keep looping
                    }
                },
                SequentialAction(
                    AutoPoints.AprilTagBlue.runToExact(),
                    robot.CheckMotif(),
                    robot.OffCamera(),
                    robot.StartShooter,
                    AutoPoints.LaunchBlue.runToExact(),
                    robot.AutoShoot(),
                    object : Action {
                        var nextAction: Action? = null

                        override fun run(p: TelemetryPacket): Boolean {
                            if (nextAction == null) {
                                nextAction = when (robot.motif) {
                                    1 -> SequentialAction(
                                        AutoPoints.PreIntakeGPP.runToFast(),
                                        robot.StartIntake,
                                        AutoPoints.GPPIntake.runToExact(),
                                        robot.WaitAction(200.0),
                                        robot.StopIntake,
                                        AutoPoints.GPPMidPoint.runToFast()
                                    )

                                    2 -> SequentialAction(
                                        AutoPoints.PreIntakePGP.runToFast(),
                                        robot.StartIntake,
                                        AutoPoints.PGPIntake.runToExact(),
                                        robot.WaitAction(200.0),
                                        robot.StopIntake,
                                        AutoPoints.PGPMidPoint.runToFast()
                                    )

                                    else -> SequentialAction(
                                        AutoPoints.PreIntakePPG.runToFast(),
                                        robot.StartIntake,
                                        AutoPoints.PPGIntake.runToExact(),
                                        robot.WaitAction(200.0),
                                        robot.StopIntake,
                                    )
                                }
                            }

                            // run the generated action normally
                            return nextAction!!.run(p)
                        }
                    },
                    robot.StartShooter,
                    AutoPoints.LaunchBlue.runToExact(),
                    robot.AutoShoot(),
                    AutoPoints.EndBlue.runToExact()
                )
            )
        )

    }
}
