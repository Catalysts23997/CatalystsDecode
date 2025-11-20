package org.firstinspires.ftc.teamcode.Competition_Code.Auto.OpModes;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.Competition_Code.Actions.Comp1Actions
import com.acmerobotics.roadrunner.Action

import org.firstinspires.ftc.teamcode.Competition_Code.Auto.AutoPoints
import org.firstinspires.ftc.teamcode.Competition_Code.Auto.RunToExactForever
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Drivetrain
import org.firstinspires.ftc.teamcode.Competition_Code.PinpointLocalizer.Localizer
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Servo
import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.Poses

@Autonomous(name = "BlueAuto", group = "Auto")
class BlueAuto : LinearOpMode() {

    companion object{
        var rT = Poses(-39.0,63.0,0.0)
        var endPos = Poses(0.0,0.0,0.0)

    }



    override fun runOpMode() {
        var k = 1.0
        rT = Poses(-39.0,63.0,0.0)

        val localizer = Localizer(hardwareMap, rT)
        val drive = Drivetrain(hardwareMap)
        val robot = Comp1Actions(hardwareMap, telemetry)

        localizer.update()
        robot.servo.state = Servo.State.HOLD
        robot.update()

        waitForStart()

        runBlocking(
            ParallelAction(
                object : Action {
                    override fun run(p: TelemetryPacket): Boolean {
                        localizer.update()
                        RunToExactForever(rT, k)
                        endPos = Poses(Localizer.pose.x, Localizer.pose.y, Localizer.pose.heading)
                        telemetry.addData("hello", rT)
                        telemetry.addData("heading", Localizer.pose.heading)
                        telemetry.addData("x", Localizer.pose.x)
                        telemetry.addData("y", Localizer.pose.y)
                        telemetry.update()
                        robot.update()
                        return true // keep looping
                    }
                },
                SequentialAction(
                    AutoPoints.AprilTagBlue.runToExact,
                    robot.CheckMotif(),
                    robot.OffCamera(),
                    robot.StartShooter,
                    AutoPoints.LaunchBlue.runToExact,
                    robot.AutoShoot(),
                    object : Action {
                        var nextAction: Action? = null

                        override fun run(p: TelemetryPacket): Boolean {
                            if (nextAction == null) {
                                nextAction = when (robot.motif) {
                                    1 -> SequentialAction(
                                        AutoPoints.PreIntakeGPP.runToExact,
                                        object: Action {
                                            override fun run(p: TelemetryPacket): Boolean {
                                                k = 0.5
                                                return false
                                            }

                                        },
                                        ParallelAction(
                                            SequentialAction(
                                                robot.Balls(),
                                                object: Action {
                                                    override fun run(p: TelemetryPacket): Boolean {
                                                        k = 1.0
                                                        return false
                                                    }

                                                }
                                            ),
                                            AutoPoints.GPPIntake3.runToExact
                                        ),
                                        AutoPoints.GPPMidPoint.runToExact
                                    )

                                    2 -> SequentialAction(
                                        AutoPoints.PreIntakePGP.runToExact,
                                        object: Action {
                                            override fun run(p: TelemetryPacket): Boolean {
                                                k = 0.5
                                                return false
                                            }

                                        },
                                        ParallelAction(
                                            SequentialAction(
                                                robot.Balls(),
                                                object: Action {
                                                    override fun run(p: TelemetryPacket): Boolean {
                                                        k = 1.0
                                                        return false
                                                    }

                                                }
                                            ),
                                            AutoPoints.PGPIntake3.runToExact,
                                        ),
                                        AutoPoints.PGPIntake2.runToExact,
                                        AutoPoints.PGPMidPoint.runToExact
                                    )

                                    else -> SequentialAction(
                                        AutoPoints.PreIntakePPG.runToExact,
                                        object: Action {
                                            override fun run(p: TelemetryPacket): Boolean {
                                                k = 0.5
                                                return false
                                            }

                                        },
                                        ParallelAction(
                                            SequentialAction(
                                                robot.Balls(),
                                                object: Action {
                                                    override fun run(p: TelemetryPacket): Boolean {
                                                        k = 1.0
                                                        return false
                                                    }

                                                }
                                            ),
                                            AutoPoints.PPGIntake3.runToExact,
                                        ),
                                    )
                                }
                            }

                            // run the generated action normally
                            return nextAction!!.run(p)
                        }
                    },
                    robot.StartShooter,
                    AutoPoints.LaunchBlue.runToExact,
                    robot.AutoShoot(),
                    AutoPoints.EndBlue.runToExact
                )
            )
        )

    }
}
