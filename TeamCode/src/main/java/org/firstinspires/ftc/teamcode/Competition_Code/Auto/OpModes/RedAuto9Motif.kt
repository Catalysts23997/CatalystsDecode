package org.firstinspires.ftc.teamcode.Competition_Code.Auto.OpModes;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.Competition_Code.Actions.Comp1Actions
import org.firstinspires.ftc.teamcode.Competition_Code.Actions.Comp2Actions
import org.firstinspires.ftc.teamcode.Competition_Code.Auto.AutoGlobals
import org.firstinspires.ftc.teamcode.Competition_Code.Auto.AutoPoints
import org.firstinspires.ftc.teamcode.Competition_Code.Auto.RunToExactForever
import org.firstinspires.ftc.teamcode.Competition_Code.PinpointLocalizer.Localizer
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Drivetrain
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Servo
import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.Poses

@Autonomous(name = "RedAuto9Motif", group = "Auto")
class RedAuto9Motif : LinearOpMode() {

    override fun runOpMode() {
        AutoGlobals.targetRobotPositon = AutoPoints.StartRed.pose

        val localizer = Localizer(hardwareMap, AutoGlobals.targetRobotPositon)
        val drive = Drivetrain(hardwareMap, Drivetrain.Alliance.Red)
        val robot = Comp2Actions(hardwareMap, telemetry)

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
                        AutoGlobals.locationOfRobot =
                            Poses(Localizer.pose.x, Localizer.pose.y, Localizer.pose.heading)
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
                    AutoPoints.AprilTagRed.runToExact(),
                    robot.CheckMotif(),
                    robot.OffCamera(),
                    robot.StartShooter,
                    AutoPoints.LaunchRed.runToExact(),
                    robot.AutoShoot(),
                    object : Action {
                        var nextAction: Action? = null

                        override fun run(p: TelemetryPacket): Boolean {
                            if (robot.motif == 0) {
                                robot.motif = 3
                            }
                            if (nextAction == null) {
                                nextAction = when (robot.motif) {
                                    1 -> SequentialAction(
                                        AutoPoints.PreIntakeGPPRed.runToFast(),
                                        robot.StartIntake,
                                        AutoPoints.GPPIntakeRed.runToExact(),
                                        robot.WaitAction(200.0),
                                        robot.StopIntake,
                                        AutoPoints.GPPMidPointRed.runToFast()
                                    )

                                    2 -> SequentialAction(
                                        AutoPoints.PreIntakePGPRed.runToFast(),
                                        robot.StartIntake,
                                        AutoPoints.PGPIntakeRed.runToExact(),
                                        robot.WaitAction(200.0),
                                        robot.StopIntake,
                                        AutoPoints.PGPMidPointRed.runToFast()
                                    )

                                    else -> SequentialAction(
                                        AutoPoints.PreIntakePPGRed.runToFast(),
                                        robot.StartIntake,
                                        AutoPoints.PPGIntakeRed.runToExact(),
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
                    AutoPoints.LaunchRed.runToExact(),
                    robot.AutoShoot(),

                    object : Action {
                        var nextAction: Action? = null

                        override fun run(p: TelemetryPacket): Boolean {
                            if (nextAction == null) {
                                nextAction = when (robot.motif) {
                                    3 -> SequentialAction(
                                        AutoPoints.PreIntakePGPRed.runToFast(),
                                        robot.StartIntake,
                                        AutoPoints.PGPIntakeRed.runToExact(),
                                        robot.WaitAction(200.0),
                                        robot.StopIntake,
                                        AutoPoints.PGPMidPointRed.runToFast()
                                    )

                                    else -> SequentialAction(
                                        AutoPoints.PreIntakePPGRed.runToFast(),
                                        robot.StartIntake,
                                        AutoPoints.PPGIntakeRed.runToExact(),
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
                    AutoPoints.LaunchRed.runToExact(),
                    robot.AutoShoot(),
                    AutoPoints.EndRed.runToExact()

                )
            )
        )

    }
}
