package org.firstinspires.ftc.teamcode.Competition_Code.Auto.OpModes;

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
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Drivetrain
import org.firstinspires.ftc.teamcode.Competition_Code.PinpointLocalizer.Localizer
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Servo
import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.Poses

@Disabled
@Autonomous(name = "BlueFar3", group = "Auto")
class BlueFar3 : LinearOpMode() {

    override fun runOpMode() {
        AutoGlobals.targetRobotPositon = AutoPoints.StartFarBlue.pose

        val localizer = Localizer(hardwareMap, AutoGlobals.targetRobotPositon)
        val drive = Drivetrain(hardwareMap, AllianceColor.Blue)
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
                {
                    if (isStopRequested) {
                        stop()
                    }
                    localizer.update()
                    RunToExactForever(AutoGlobals.targetRobotPositon)
                    AutoGlobals.locationOfRobot = Poses(Localizer.pose.x, Localizer.pose.y, Localizer.pose.heading)
                    telemetry.addData("hello", AutoGlobals.targetRobotPositon)
                    telemetry.addData("df", Localizer.pose.heading)
                    telemetry.addData("x", Localizer.pose.x)
                    telemetry.addData("y", Localizer.pose.y)
                    telemetry.update()
                    robot.update()
                    true
                },
                SequentialAction(
                    robot.StartShooter,
                    robot.WaitAction(1000.0),
                    robot.StartIntake,


                    AutoPoints.LaunchFarBlue.runToExact(),
                    robot.ShootFar(),
                    AutoPoints.MoveFarBlue.runToExact()
                )
            )
        )

    }
}
