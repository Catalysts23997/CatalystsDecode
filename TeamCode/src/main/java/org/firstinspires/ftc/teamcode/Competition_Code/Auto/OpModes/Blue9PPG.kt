package org.firstinspires.ftc.teamcode.Competition_Code.Auto.OpModes

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
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

@Disabled
@Autonomous(name = "Blue9PPG", group = "Auto")
class Blue9PPG : LinearOpMode() {

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
                        telemetry.addData("Drive speed", AutoGlobals.driveSpeed)
                        telemetry.update()
                        robot.update()

                        return true // keep looping
                    }
                },
                SequentialAction(
                    robot.StartShooter,
                    robot.StartIntake,
                    AutoPoints.LaunchBlue.runToExact(),
                    robot.Shoot(),

                    AutoPoints.PrePPGBlue.runToFast(),
                    robot.StartIntake,
                    AutoPoints.PPGBlue.runToExact(),

                    AutoPoints.EjectBlue.runToExact(),
                    robot.EjectOne(),
                    AutoPoints.LaunchBlue.runToExact(),
                    robot.Shoot(),

                    AutoPoints.PreGPPBlue.runToFast(),
                    robot.StartIntake,
                    AutoPoints.GPPBlue.runToExact(),
                    AutoPoints.GPPBackBlue.runToFast(),

                    AutoPoints.EjectBlue.runToExact(),
                    robot.EjectOne(),
                    AutoPoints.LaunchBlue.runToExact(),
                    robot.ShootSlow(),

                    AutoPoints.PrePGPBlue.runToFast(),
                    robot.StartIntake,
                    AutoPoints.PGPBlue.runToExact(),

                    AutoPoints.LaunchBlue.runToExact(),
                    robot.ShootSlow(),
                    AutoPoints.EndBlue.runToExact()
                )
            )
        )

    }
}