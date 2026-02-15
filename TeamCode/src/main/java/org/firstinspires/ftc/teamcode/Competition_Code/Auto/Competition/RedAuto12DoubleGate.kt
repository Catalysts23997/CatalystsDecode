package org.firstinspires.ftc.teamcode.Competition_Code.Auto.Competition

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.Competition_Code.Actions.InterleagueActions
import org.firstinspires.ftc.teamcode.Competition_Code.AllianceColor
import org.firstinspires.ftc.teamcode.Competition_Code.Auto.AutoGlobals
import org.firstinspires.ftc.teamcode.Competition_Code.Auto.AutoPoints
import org.firstinspires.ftc.teamcode.Competition_Code.Auto.RunToExactForever
import org.firstinspires.ftc.teamcode.Competition_Code.PinpointLocalizer.Localizer
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Drivetrain
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Servo
import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.Poses

@Autonomous(name = "Red12DoubleGate", group = "Auto")
class RedAuto12DoubleGate : LinearOpMode() {

    override fun runOpMode() {

        val dash: FtcDashboard = FtcDashboard.getInstance()
        telemetry = dash.telemetry

        AutoGlobals.targetRobotPositon = AutoPoints.StartRed.pose

        val localizer = Localizer(hardwareMap, AutoGlobals.targetRobotPositon)
        val drive = Drivetrain(hardwareMap, AllianceColor.Red)
        val robot = InterleagueActions(hardwareMap, telemetry)

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
                        AutoGlobals.locationOfRobot = Poses(
                            Localizer.Companion.pose.x,
                            Localizer.Companion.pose.y,
                            Localizer.Companion.pose.heading
                        )

                        telemetry.addData(
                            "Target Position",
                            AutoGlobals.targetRobotPositon.toString()
                        )
                        telemetry.addData("Current Pose", Localizer.Companion.pose.toString())
                        telemetry.addData(
                            "Location of robot being transferred",
                            AutoGlobals.locationOfRobot.toString()
                        )

                        telemetry.addData("Launcher rpm goal", robot.launcher.goalRPM)
                        telemetry.addData("Drive speed", AutoGlobals.driveSpeed)
                        telemetry.update()
                        robot.update()

                        return true // keep looping
                    }
                },
                SequentialAction(
                    robot.StartShooter,
                    robot.StartIntake,
                    AutoPoints.LaunchRed.runToExact(),
                    robot.Shoot(),

                    AutoPoints.PreIntakePPGRed.runToFast(),
                    robot.StartIntake,
                    AutoPoints.PPGIntakeRed.runToExact(),
                    robot.WaitAction(125.0),

                    AutoPoints.PreGateRed.runToExact(),
                    AutoPoints.GateRed.runToExact(),
                    robot.WaitAction(600.0),

                    AutoPoints.LaunchRed.runToExact(),
                    robot.Shoot(),

                    AutoPoints.PreIntakePGPRed.runToFast(),
                    robot.StartIntake,
                    AutoPoints.PGPIntakeRed.runToExact(),
                    robot.WaitAction(125.0),

                    AutoPoints.GateMidRed.runToFast(),
                    AutoPoints.PreGateRed.runToExact(),
                    AutoPoints.GateRed.runToExact(),
                    robot.WaitAction(600.0),

                    AutoPoints.LaunchRed.runToExact(),
                    robot.Shoot(),

                    AutoPoints.PreIntakeGPPRed.runToFast(),
                    robot.StartIntake,
                    AutoPoints.GPPIntakeRed.runToExact(),
                    robot.WaitAction(125.0),

                    AutoPoints.LaunchRed.runToExact(),
                    robot.Shoot(),
                    AutoPoints.EndRed.runToExact()

                )
            )
        )

    }
}