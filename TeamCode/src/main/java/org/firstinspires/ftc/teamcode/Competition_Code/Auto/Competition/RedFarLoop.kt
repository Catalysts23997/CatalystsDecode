package org.firstinspires.ftc.teamcode.Competition_Code.Auto.Competition;

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.acmerobotics.roadrunner.Action
import org.firstinspires.ftc.teamcode.Competition_Code.Actions.InterleagueActions
import org.firstinspires.ftc.teamcode.Competition_Code.AllianceColor
import org.firstinspires.ftc.teamcode.Competition_Code.Auto.AutoGlobals

import org.firstinspires.ftc.teamcode.Competition_Code.Auto.AutoPoints
import org.firstinspires.ftc.teamcode.Competition_Code.Auto.RunToExactForever
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Drivetrain
import org.firstinspires.ftc.teamcode.Competition_Code.PinpointLocalizer.Localizer
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Servo
import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.Poses

@Autonomous(name = "RedFarLoop", group = "Auto")
class RedFarLoop : LinearOpMode() {


    override fun runOpMode() {

        val dash: FtcDashboard = FtcDashboard.getInstance()
        telemetry = dash.telemetry

        AutoGlobals.targetRobotPositon = AutoPoints.StartFarRed.pose

        val localizer = Localizer(hardwareMap, AutoGlobals.targetRobotPositon)
        val drive = Drivetrain(hardwareMap, AllianceColor.Red)
        val robot = InterleagueActions(hardwareMap, telemetry)

        sleep(100)
        localizer.update()
        robot.holder.state = Servo.State.STOP1
        robot.update()
        robot.launcher.baseRPM = 3200.0

        waitForStart()

        AutoGlobals.FarAuto = true
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
                    robot.StartShooter,
                    robot.StartIntake,

                    AutoPoints.LaunchFarRed.runToExact(),
                    robot.ShootFar(),

                    robot.StartIntake,
                    AutoPoints.PGPIntakeFarRed.runToExact(),
                    robot.WaitAction(125.0),

                    AutoPoints.LaunchFarRed.runToExact(),
                    robot.ShootFar(),

                    robot.StartIntake,
                    AutoPoints.PGPIntakeFarRed.runToExact(),
                    robot.WaitAction(125.0),

                    AutoPoints.LaunchFarRed.runToExact(),
                    robot.ShootFar(),

                    robot.StartIntake,
                    AutoPoints.PGPIntakeFarRed.runToExact(),
                    robot.WaitAction(125.0),

                    AutoPoints.LaunchFarRed.runToExact(),
                    robot.ShootFar(),

                    robot.StartIntake,
                    AutoPoints.PGPIntakeFarRed.runToExact(),
                    robot.WaitAction(125.0),

                    AutoPoints.LaunchFarRed.runToExact(),
                    robot.ShootFar(),

                    robot.StartIntake,
                    AutoPoints.PGPIntakeFarRed.runToExact(),
                )
            )
        )

    }
}
