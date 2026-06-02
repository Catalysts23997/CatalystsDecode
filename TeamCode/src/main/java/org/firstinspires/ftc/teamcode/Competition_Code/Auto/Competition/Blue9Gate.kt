package org.firstinspires.ftc.teamcode.Competition_Code.Auto.Competition;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.teamcode.Competition_Code.Actions.InterleagueActions
import org.firstinspires.ftc.teamcode.Competition_Code.AllianceColor
import org.firstinspires.ftc.teamcode.Competition_Code.Auto.AutoGlobals
import org.firstinspires.ftc.teamcode.Competition_Code.Auto.AutoPoints
import org.firstinspires.ftc.teamcode.Competition_Code.Auto.RunToExactForever
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Drivetrain
import org.firstinspires.ftc.teamcode.Competition_Code.PinpointLocalizer.Localizer
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Servo
import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.Poses

@Disabled
@Autonomous(name = "Blue9Gate", group = "Auto")
class Blue9Gate : LinearOpMode() {

    override fun runOpMode() {
        AutoGlobals.targetRobotPositon = AutoPoints.FastStartBlue.pose

        val localizer = Localizer(hardwareMap, AutoGlobals.targetRobotPositon)
        val drive = Drivetrain(hardwareMap, AllianceColor.Blue)
        val robot = InterleagueActions(hardwareMap, telemetry)

        robot.holder.state = Servo.State.STOP1
        robot.update()



        waitForStart()

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

                        telemetry.addData("Target Position", AutoGlobals.targetRobotPositon.toString())
                        telemetry.addData("Current Pose", Localizer.pose.toString())
                        telemetry.addData("Location of robot being transferred", AutoGlobals.locationOfRobot.toString())
                        telemetry.addData("Drive speed", AutoGlobals.driveSpeed)
                        telemetry.update()
                        robot.update()

                        return true // keep looping
                    }
                },
                SequentialAction(
                    robot.StartShooter,
                    AutoPoints.LaunchBlue.runToExact(),
                    robot.Shoot(),

                    AutoPoints.PrePPGBlue.runToFast(),
                    robot.StartIntake,
                    AutoPoints.PPGBlue.runToExact(),
                    robot.WaitAction(200.0),
                    robot.StopIntake,

                    AutoPoints.PreGateBlue.runToExact(),
                    AutoPoints.GateBlue.runToExact(),
                    robot.WaitAction(800.0),

                    AutoPoints.LaunchBlue.runToExact(),
                    robot.Shoot(),

                    AutoPoints.PrePGPBlue.runToFast(),
                    robot.StartIntake,
                    AutoPoints.PGPBlue.runToExact(),
                    robot.WaitAction(200.0),
                    robot.StopIntake,

                    AutoPoints.GateMidBlue.runToFast(),
                    AutoPoints.PreGateBlue.runToExact(),
                    AutoPoints.GateBlue.runToExact(),
                    robot.WaitAction(800.0),

                    AutoPoints.LaunchBlue.runToExact(),
                    robot.Shoot(),
                    AutoPoints.EndBlue.runToExact()

                )
            )
        )

    }
}
