package org.firstinspires.ftc.teamcode.Competition_Code.Auto.OpModes;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.Competition_Code.Actions.Comp1Actions
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.RaceAction
import org.firstinspires.ftc.teamcode.Competition_Code.Auto.AutoGlobals
import org.firstinspires.ftc.teamcode.Competition_Code.Auto.AutoPoints
import org.firstinspires.ftc.teamcode.Competition_Code.Auto.RunToExactForever
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Drivetrain
import org.firstinspires.ftc.teamcode.Competition_Code.PinpointLocalizer.Localizer
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Servo
import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.Poses

@Autonomous(name = "BlueAuto9", group = "Auto")
class BlueAuto9 : LinearOpMode() {

    override fun runOpMode() {
        AutoGlobals.targetRobotPositon = AutoPoints.StartBlue.pose
        var powerCoefficient = 1.0

        val localizer = Localizer(hardwareMap, AutoGlobals.targetRobotPositon)
        val drive = Drivetrain(hardwareMap)
        val robot = Comp1Actions(hardwareMap, telemetry)

        localizer.update()
        robot.holder.state = Servo.State.STOP
        robot.update()

        waitForStart()

        AutoGlobals.AutonomousRan = true

        runBlocking(
            ParallelAction(
                object : Action {
                    override fun run(p: TelemetryPacket): Boolean {
                        localizer.update()
                        RunToExactForever(AutoGlobals.targetRobotPositon, powerCoefficient)
                        AutoGlobals.locationOfRobot = Poses(Localizer.pose.x, Localizer.pose.y, 0.0)

                        telemetry.addData("hello", AutoGlobals.targetRobotPositon)
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
                            if(robot.motif == 0){
                                robot.motif = 3
                            }
                            if (nextAction == null) {
                                nextAction = when (robot.motif) {
                                    1 -> SequentialAction(
                                        AutoPoints.PreIntakeGPP.runToExact,
                                        object: Action {
                                            override fun run(p: TelemetryPacket): Boolean {
                                                powerCoefficient = 0.5
                                                return false
                                            }

                                        },
                                        RaceAction(
                                            SequentialAction(
                                                robot.BallsIntake(),
                                                object : Action {
                                                    override fun run(p: TelemetryPacket): Boolean {
                                                        powerCoefficient = 1.0
                                                        return false
                                                    }

                                                },
                                            ),
                                            AutoPoints.GPPIntake.runToExact,
                                        ),
                                        AutoPoints.GPPMidPoint.runToExact
                                    )

                                    2 -> SequentialAction(
                                        AutoPoints.PreIntakePGP.runToExact,
                                        object: Action {
                                            override fun run(p: TelemetryPacket): Boolean {
                                                powerCoefficient = 0.5
                                                return false
                                            }

                                        },
                                        RaceAction(
                                            SequentialAction(
                                                robot.BallsIntake(),
                                                object: Action {
                                                    override fun run(p: TelemetryPacket): Boolean {
                                                        powerCoefficient = 1.0
                                                        return false
                                                    }

                                                },
                                            ),
                                            AutoPoints.PGPIntake.runToExact,
                                        ),
                                        AutoPoints.PGPMidPoint.runToExact
                                    )

                                    else -> SequentialAction(
                                        AutoPoints.PreIntakePPG.runToExact,
                                        object: Action {
                                            override fun run(p: TelemetryPacket): Boolean {
                                                powerCoefficient = 0.5
                                                return false
                                            }

                                        },
                                        RaceAction(
                                            SequentialAction(
                                                robot.BallsIntake(),
                                                object: Action {
                                                    override fun run(p: TelemetryPacket): Boolean {
                                                        powerCoefficient = 1.0
                                                        return false
                                                    }

                                                },
                                            ),
                                            AutoPoints.PPGIntake.runToExact,
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

                    object : Action {
                        var nextAction: Action? = null

                        override fun run(p: TelemetryPacket): Boolean {
                            if (nextAction == null) {
                                nextAction = when (robot.motif) {
                                    3 -> SequentialAction(
                                        AutoPoints.PreIntakePGP.runToExact,
                                        object: Action {
                                            override fun run(p: TelemetryPacket): Boolean {
                                                powerCoefficient = 0.5
                                                return false
                                            }

                                        },
                                        RaceAction(
                                            SequentialAction(
                                                robot.BallsIntake(),
                                                object: Action {
                                                    override fun run(p: TelemetryPacket): Boolean {
                                                        powerCoefficient = 1.0
                                                        return false
                                                    }
                                                },
                                            ),
                                            AutoPoints.PGPIntake.runToExact,
                                        ),
                                        AutoPoints.PGPMidPoint.runToExact
                                    )
                                    else -> SequentialAction(
                                        AutoPoints.PreIntakePPG.runToExact,
                                        object: Action {
                                            override fun run(p: TelemetryPacket): Boolean {
                                                powerCoefficient = 0.5
                                                return false
                                            }

                                        },
                                        RaceAction(
                                            SequentialAction(
                                            robot.BallsIntake(),
                                                object: Action {
                                                    override fun run(p: TelemetryPacket): Boolean {
                                                        powerCoefficient = 1.0
                                                        return false
                                                    }

                                                },
                                            ),
                                            AutoPoints.PPGIntake.runToExact,
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
