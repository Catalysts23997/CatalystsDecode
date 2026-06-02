package org.firstinspires.ftc.teamcode.Competition_Code.Auto.Competition;

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

@Autonomous(name = "Red9Gate", group = "Auto")
class Red9Gate : LinearOpMode() {

    override fun runOpMode() {
        AutoGlobals.targetRobotPositon = AutoPoints.FastStartRed.pose

        val localizer = Localizer(hardwareMap, AutoGlobals.targetRobotPositon)
        val drive = Drivetrain(hardwareMap, AllianceColor.Red)
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

                    AutoPoints.LaunchRed.runToExact(),
                    robot.Shoot(),


                    AutoPoints.PrePPGRed.runToFast(),
                    robot.StartIntake,
                    AutoPoints.PPGRed.runToExact(),
                    robot.WaitAction(200.0),
                    robot.StopIntake,

                    AutoPoints.PreGateRed.runToExact(),
                    AutoPoints.GateRed.runToExact(),
                    robot.WaitAction(800.0),

                    AutoPoints.LaunchRed.runToExact(),
                    robot.Shoot(),

                    AutoPoints.PrePGPRed.runToFast(),
                    robot.StartIntake,
                    AutoPoints.PGPRed.runToExact(),
                    robot.WaitAction(200.0),
                    robot.StopIntake,

                    AutoPoints.GateMidRed.runToFast(),
                    AutoPoints.PreGateRed.runToExact(),
                    AutoPoints.GateRed.runToExact(),
                    robot.WaitAction(800.0),

                    AutoPoints.LaunchRed.runToExact(),
                    robot.Shoot(),
                    AutoPoints.EndRed.runToExact()

                )
            )
        )

    }
}
