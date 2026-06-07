package org.firstinspires.ftc.teamcode.Competition_Code.Auto.Competition;

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.acmerobotics.roadrunner.Action
import net.mccoder.ftvision.processors.ScanDirection
import org.firstinspires.ftc.teamcode.Competition_Code.Actions.InterleagueActions
import org.firstinspires.ftc.teamcode.Competition_Code.AllianceColor
import org.firstinspires.ftc.teamcode.Competition_Code.Auto.AutoGlobals

import org.firstinspires.ftc.teamcode.Competition_Code.Auto.AutoPoints
import org.firstinspires.ftc.teamcode.Competition_Code.Auto.RunToExactForever
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Drivetrain
import org.firstinspires.ftc.teamcode.Competition_Code.PinpointLocalizer.Localizer
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Servo
import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.Poses

@Autonomous(name = "BallDetectorRedFarPartner", group = "Auto")
class BallDetectorRedFarPartner : LinearOpMode() {


    override fun runOpMode() {

        val dash: FtcDashboard = FtcDashboard.getInstance()
        telemetry = dash.telemetry

        AutoGlobals.targetRobotPositon = AutoPoints.StartFarRed.pose

        val localizer = Localizer(hardwareMap, AutoGlobals.targetRobotPositon)
        val drive = Drivetrain(hardwareMap, AllianceColor.Red)
        val robot = InterleagueActions(hardwareMap, telemetry)

        robot.holder.state = Servo.State.STOP1
        robot.update()
        robot.launcher.baseRPM = 3150.0

        // start camera
        robot.initFTVision(ScanDirection.Right)

        waitForStart()

        AutoGlobals.FarAuto = true
        AutoGlobals.AutonomousRan = true


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

                    AutoPoints.LaunchFarRed.runToExact(),
                    robot.ShootFar(),

                    AutoPoints.PreGPPFarRed.runToFast(),
                    robot.StartIntake,
                    AutoPoints.GPPFarRed.runToExact(),
                    robot.StopIntake,

                    AutoPoints.LaunchFarRed.runToExact(),
                    robot.ShootFar(),

                    // BEGIN BALL DETECTOR
                    robot.StopShooter,
                    AutoPoints.BallDetectorWaitRed.runToExact(),

                    // ball detector
                    robot.BallDetector { point -> point.x >= 198.0 && point.y >= 180 },

                    AutoPoints.BallDetectorWaitRed.runToExact(),
                    robot.ShootFar(),

                    // move out of the way
                    // we don't care if this is skipped
                    AutoPoints.BallDetectorWaitRed.runToExact()
                )
            )
        )

    }
}
