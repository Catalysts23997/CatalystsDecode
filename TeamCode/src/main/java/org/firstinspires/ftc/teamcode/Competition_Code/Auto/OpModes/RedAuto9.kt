package org.firstinspires.ftc.teamcode.Competition_Code.Auto.OpModes;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.Competition_Code.Actions.Comp1Actions
import com.acmerobotics.roadrunner.Action

import org.firstinspires.ftc.teamcode.Competition_Code.Auto.OpModes.BlueAuto.Companion.rT
import org.firstinspires.ftc.teamcode.Competition_Code.Auto.OpModes.BlueAuto.Companion.endPos

import org.firstinspires.ftc.teamcode.Competition_Code.Auto.AutoPoints
import org.firstinspires.ftc.teamcode.Competition_Code.Auto.RunToExactForever
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Drivetrain
import org.firstinspires.ftc.teamcode.Competition_Code.PinpointLocalizer.Localizer
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Servo
import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.Poses

@Autonomous(name = "RedAuto9", group = "Auto")
class RedAuto9 : LinearOpMode() {





    override fun runOpMode() {
        rT = AutoPoints.StartRed.pose

        var k = 1.0

        val localizer = Localizer(hardwareMap, rT)
        val drive = Drivetrain(hardwareMap)
        val robot = Comp1Actions(hardwareMap, telemetry)

        localizer.update()
        robot.holder.state = Servo.State.STOP
        robot.update()

        waitForStart()

        runBlocking(
            ParallelAction(
                object : Action {
                    override fun run(p: TelemetryPacket): Boolean {
                        localizer.update()
                        RunToExactForever(rT, k)
                        endPos = Poses(Localizer.pose.x, Localizer.pose.y, 0.0)
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
                    AutoPoints.AprilTagRed.runToExact,
                    robot.CheckMotif(),
                    robot.OffCamera(),
                    robot.StartShooter,
                    AutoPoints.LaunchRed.runToExact,
                    robot.AutoShoot(),
                    object : Action {
                        var nextAction: Action? = null

                        override fun run(p: TelemetryPacket): Boolean {
                            if(robot.motif == 0){
                                robot.motif = 3
                            }
                            if (nextAction == null) {
                                nextAction = when (robot.motif) {
                                    1 -> SequentialAction(
                                        AutoPoints.PreIntakeGPPRed.runToExact,
                                        object: Action {
                                            override fun run(p: TelemetryPacket): Boolean {
                                                k = 0.5
                                                return false
                                            }

                                        },
                                        ParallelAction(
                                            SequentialAction(
                                            robot.BallsIntake(),
                                                object: Action {
                                                    override fun run(p: TelemetryPacket): Boolean {
                                                        k = 1.0
                                                        return false
                                                    }

                                                },
                                            ),
                                            AutoPoints.GPPIntake3Red.runToExact,
                                        ),
                                        AutoPoints.GPPIntake2Red.runToExact,
                                        AutoPoints.GPPMidPointRed.runToExact
                                    )

                                    2 -> SequentialAction(
                                        AutoPoints.PreIntakePGPRed.runToExact,
                                        object: Action {
                                            override fun run(p: TelemetryPacket): Boolean {
                                                k = 0.5
                                                return false
                                            }

                                        },
                                        ParallelAction(
                                            SequentialAction(
                                            robot.BallsIntake(),
                                                object: Action {
                                                    override fun run(p: TelemetryPacket): Boolean {
                                                        k = 1.0
                                                        return false
                                                    }

                                                },
                                            ),

                                            AutoPoints.PGPIntake3Red.runToExact,

                                        ),
                                        AutoPoints.PGPIntake2Red.runToExact,
                                        AutoPoints.PGPMidPointRed.runToExact
                                    )

                                    else -> SequentialAction(
                                        AutoPoints.PreIntakePPGRed.runToExact,
                                        object: Action {
                                            override fun run(p: TelemetryPacket): Boolean {
                                                k = 0.5
                                                return false
                                            }

                                        },
                                        ParallelAction(
                                            SequentialAction(
                                            robot.BallsIntake(),
                                                object: Action {
                                                    override fun run(p: TelemetryPacket): Boolean {
                                                        k = 1.0
                                                        return false
                                                    }

                                                },
                                            ),

                                            AutoPoints.PPGIntake3Red.runToExact
                                        ),
                                    )
                                }
                            }

                            // run the generated action normally
                            return nextAction!!.run(p)
                        }
                    },
                    robot.StartShooter,
                    AutoPoints.LaunchRed.runToExact,
                    robot.AutoShoot(),

                    object : Action {
                        var nextAction: Action? = null

                        override fun run(p: TelemetryPacket): Boolean {
                            if (nextAction == null) {
                                nextAction = when (robot.motif) {
                                    3 -> SequentialAction(
                                        AutoPoints.PreIntakePGPRed.runToExact,
                                        object: Action {
                                            override fun run(p: TelemetryPacket): Boolean {
                                                k = 0.5
                                                return false
                                            }

                                        },
                                        ParallelAction(
                                            SequentialAction(
                                            robot.BallsIntake(),
                                                object: Action {
                                                    override fun run(p: TelemetryPacket): Boolean {
                                                        k = 1.0
                                                        return false
                                                    }

                                                }
                                            ),
                                            AutoPoints.PGPIntake3Red.runToExact

                                        ),
                                        AutoPoints.PGPIntake2Red.runToExact,
                                        AutoPoints.PGPMidPointRed.runToExact
                                    )
                                    else -> SequentialAction(
                                        AutoPoints.PreIntakePPGRed.runToExact,
                                        object: Action {
                                            override fun run(p: TelemetryPacket): Boolean {
                                                k = 0.5
                                                return false
                                            }

                                        },
                                        ParallelAction(
                                            SequentialAction(
                                                robot.BallsIntake(),
                                                object: Action {
                                                    override fun run(p: TelemetryPacket): Boolean {
                                                        k = 1.0
                                                        return false
                                                    }

                                                },
                                            ),
                                            AutoPoints.PPGIntake3Red.runToExact

                                        ),
                                    )
                                }
                            }

                            // run the generated action normally
                            return nextAction!!.run(p)
                        }
                    },
                    robot.StartShooter,
                    AutoPoints.LaunchRed.runToExact,
                    robot.AutoShoot(),
                    AutoPoints.EndRed.runToExact

                )
            )
        )

    }
}
