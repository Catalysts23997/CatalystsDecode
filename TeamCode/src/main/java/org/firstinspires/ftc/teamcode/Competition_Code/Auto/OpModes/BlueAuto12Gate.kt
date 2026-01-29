package org.firstinspires.ftc.teamcode.Competition_Code.Auto.OpModes;

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.acmerobotics.roadrunner.Action
import org.firstinspires.ftc.teamcode.Competition_Code.Actions.Comp2Actions
import org.firstinspires.ftc.teamcode.Competition_Code.Actions.InterleagueActions
import org.firstinspires.ftc.teamcode.Competition_Code.AllianceColor
import org.firstinspires.ftc.teamcode.Competition_Code.Auto.AutoGlobals
import org.firstinspires.ftc.teamcode.Competition_Code.Auto.AutoPoints
import org.firstinspires.ftc.teamcode.Competition_Code.Auto.RunToExactForever
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Drivetrain
import org.firstinspires.ftc.teamcode.Competition_Code.PinpointLocalizer.Localizer
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Servo
import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.Poses

@Autonomous(name = "BlueAuto12Gate", group = "Auto")
class BlueAuto12Gate : LinearOpMode() {

    override fun runOpMode() {
        AutoGlobals.targetRobotPositon = AutoPoints.StartBlue.pose

        val dash: FtcDashboard = FtcDashboard.getInstance()
        telemetry = dash.telemetry

        val localizer = Localizer(hardwareMap, AutoGlobals.targetRobotPositon)
        val drive = Drivetrain(hardwareMap, AllianceColor.Blue)
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
                        AutoGlobals.locationOfRobot = Poses(Localizer.pose.x, Localizer.pose.y, Localizer.pose.heading)

                        telemetry.addData("Target Position", AutoGlobals.targetRobotPositon.toString())
                        telemetry.addData("Current Pose", Localizer.pose.toString())
                        telemetry.addData("Location of robot being transferred", AutoGlobals.locationOfRobot.toString())
                        telemetry.addData("Drive speed", AutoGlobals.driveSpeed)
                        telemetry.addData("Launcher rpm goal", robot.launcher.goalRPM)

                        telemetry.update()
                        robot.update()

                        return true // keep looping
                    }
                },
                SequentialAction(
                    robot.StartShooter,
                    AutoPoints.LaunchBlue.runToExact(),
                    robot.Shoot(),

                    AutoPoints.PreIntakePPG.runToFast(),
                    robot.StartIntake,
                    AutoPoints.PPGIntake.runToExact(),
                    robot.WaitAction(200.0),
                    robot.StopIntake,

                    AutoPoints.PreGate.runToExact(),
                    AutoPoints.Gate.runToExact(),
                    robot.WaitAction(400.0),

                    AutoPoints.LaunchBlue.runToExact(),
                    robot.Shoot(),

                    AutoPoints.PreIntakePGP.runToFast(),
                    robot.StartIntake,
                    AutoPoints.PGPIntake.runToExact(),
                    robot.WaitAction(200.0),
                    robot.StopIntake,
                    AutoPoints.PGPMidPoint.runToFast(),

                    AutoPoints.LaunchBlue.runToExact(),
                    robot.Shoot(),

                    AutoPoints.PreIntakeGPP.runToFast(),
                    robot.StartIntake,
                    AutoPoints.GPPIntake.runToExact(),
                    robot.WaitAction(200.0),
                    robot.StopIntake,
                    AutoPoints.GPPMidPoint.runToFast(),

                    AutoPoints.LaunchBlue.runToExact(),
                    robot.Shoot(),
                    AutoPoints.EndBlue.runToExact()

                )
            )
        )

    }
}
