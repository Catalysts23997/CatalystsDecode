package org.firstinspires.ftc.teamcode.Competition_Code.Tele.OpModes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Competition_Code.Actions.Comp2Actions
import org.firstinspires.ftc.teamcode.Competition_Code.Auto.AutoGlobals
import org.firstinspires.ftc.teamcode.Competition_Code.Auto.AutoPoints
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.LauncherPoint
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Drivetrain
import org.firstinspires.ftc.teamcode.Competition_Code.PinpointLocalizer.Localizer
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.DrivetrainOverride
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Intake
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Servo
import org.firstinspires.ftc.teamcode.Competition_Code.Tele.TeleGlobals
import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.goalAngle

@TeleOp(name = "BlueTele", group = "Linear OpMode")
class BlueTele : LinearOpMode() {

    // get the current launcher point
    var currentLaunchPointIndex = LauncherPoint.getPriorityPoint(LauncherPoint.blueLauncherPoints)
    var currentLaunchPoint: LauncherPoint = LauncherPoint.blueLauncherPoints[currentLaunchPointIndex]
    lateinit var robot: Comp2Actions

    var driveShouldRotate = false

    override fun runOpMode() {
        val dash: FtcDashboard = FtcDashboard.getInstance()
//        telemetry = dash.telemetry

        if(AutoGlobals.AutonomousRan) {
            TeleGlobals.currentPosition = AutoGlobals.locationOfRobot!!
        } else {

            TeleGlobals.currentPosition = AutoPoints.StartBlue.pose
        }

        telemetry.addData("Robot at position ",  TeleGlobals.currentPosition)

        val packet = TelemetryPacket()
        var runningActions = ArrayList<Action>()

        val buttonDebounce = 200 // ms minimum between button presses
        val buttonTimer = ElapsedTime()

        robot = Comp2Actions(hardwareMap, telemetry)

        val drive = Drivetrain(hardwareMap, Drivetrain.Alliance.Blue)
        val localizer = Localizer(hardwareMap, TeleGlobals.currentPosition)

        telemetry.addData("Robot at zero:",  Localizer.pose)
        localizer.update()
        telemetry.addData("Robot at original position:",  Localizer.pose)

        val driveOverride = DrivetrainOverride()

        /**
         * This is only used for telemetry, nothing more
         */
        var driveOverrideSafetyTimer = 0L
        robot.holder.state = Servo.State.STOP1

        telemetry.update()

        waitForStart()

        localizer.update()
        localizer.transferToTele()

        telemetry.clear()
        buttonTimer.reset()

        while (opModeIsActive()) {
            val intaking = robot.intake.state == Intake.State.INTAKING
            val reversing = robot.intake.state == Intake.State.REVERSE

            if (gamepad1.right_trigger >= 0.5 && buttonTimer.milliseconds() >= buttonDebounce && runningActions.isEmpty()) {

                runningActions.add(robot.ShootThrough())
                buttonTimer.reset()
            }

            if (gamepad1.left_trigger >=0.5 && buttonTimer.milliseconds() >= buttonDebounce) {
                if(!intaking){
                    runningActions.add(robot.StartIntake)
                }
                else{
                    runningActions.add(robot.StopIntake)
                }

                buttonTimer.reset()
            }

            if (gamepad1.right_bumper && buttonTimer.milliseconds() >= buttonDebounce){
                robot.launcher.change +=100
                buttonTimer.reset()
            }
            if (gamepad1.left_bumper && buttonTimer.milliseconds() >= buttonDebounce){
                robot.launcher.change -=100
                buttonTimer.reset()
            }

            if (gamepad1.dpad_down && buttonTimer.milliseconds() >= buttonDebounce) {
                if(!reversing){
                    runningActions.add(robot.ReverseIntake)
                }
                else{
                    runningActions.add(robot.StopIntake)
                }

                buttonTimer.reset()
            }

            if (gamepad1.dpad_up && buttonTimer.milliseconds() >= buttonDebounce){
                currentLaunchPointIndex = 0
                currentLaunchPoint = LauncherPoint.blueLauncherPoints[currentLaunchPointIndex]
                robot.launcher.baseRPM = currentLaunchPoint.launcherRPM
                buttonTimer.reset()
            }

            if (gamepad1.dpad_right && buttonTimer.milliseconds() >= buttonDebounce) {
                cycleLauncherPoint(true)
                buttonTimer.reset()
            } else if (gamepad1.dpad_left && buttonTimer.milliseconds() >= buttonDebounce) {
                cycleLauncherPoint(false)
                buttonTimer.reset()
            }

            if (gamepad1.y && buttonTimer.milliseconds() >= buttonDebounce) {
                driveOverride.beginOverriding(currentLaunchPoint.pose)
                buttonTimer.reset()
            }

            if(gamepad1.a ){
                localizer.resetOdo()
            }

            if (gamepad1.right_stick_button && buttonTimer.milliseconds() >= buttonDebounce) {
                // when changing modes in the drivetrain override,
                // just stop overriding
                driveOverride.stopOverriding()

                driveShouldRotate = !driveShouldRotate
                buttonTimer.reset()
            }


            //testing
            if (gamepad2.right_bumper && buttonTimer.milliseconds() >= buttonDebounce){
                robot.holder.launchpos +=0.01
                buttonTimer.reset()

            }
            if (gamepad2.left_bumper && buttonTimer.milliseconds() >= buttonDebounce) {
                robot.holder.launchpos -= 0.01
                buttonTimer.reset()

            }


            // updatePID running actions
            val newActions = ArrayList<Action>()
            runningActions.forEach {
                it.preview(packet.fieldOverlay())
                if (it.run(packet)) {
                    newActions.add(it)
                }
            }
            runningActions = newActions

            TeleGlobals.currentPosition = Localizer.pose

            //updatePID subsystems

            localizer.update()
            robot.update()


            //drivetrain overrides

            if (driveOverride.shouldOverrideInput()) {
                if (driveOverride.safetyMeasures(gamepad1)) {
                    driveOverrideSafetyTimer = System.currentTimeMillis()
                }

                driveOverride.update(drive)
            } else if(driveShouldRotate){
                val launchAngle = goalAngle(Localizer.pose.x, Localizer.pose.y, Drivetrain.Alliance.Blue)
                driveOverride.rotate(
                    drive, launchAngle, gamepad1
                )
            }
            else {
                drive.update(
                    arrayListOf(
                        gamepad1.left_stick_x,
                        -gamepad1.left_stick_y,
                        gamepad1.right_stick_x
                    )
                )
            }


            //telemetry
            val overrideTimeLeft = System.currentTimeMillis() - driveOverrideSafetyTimer
            if (overrideTimeLeft < 5000) {
                telemetry.addData("Drive train override safety was tripped!", overrideTimeLeft)
            }

            telemetry.addData("Current Launcher Point", currentLaunchPoint.displayName)
            telemetry.addData("Launcher rpm goal", robot.launcher.goalRPM)
            telemetry.addData("Launcher at RPM?", robot.launcher.atTargetRPM(robot.launcher.goalRPM, 100.0))
            telemetry.addData("servopos", robot.holder.launchpos)

            telemetry.addData("Current Pose", Localizer.pose.toString())
            telemetry.addData("Motor Power", robot.launcher.rightLauncher.power)
            telemetry.addData("Motor Velo", robot.launcher.rightLauncher.velocity)

            telemetry.update()
        }
    }

    fun cycleLauncherPoint(forwards: Boolean) {
        if (forwards) {
            currentLaunchPointIndex += 1

            if (currentLaunchPointIndex >= LauncherPoint.blueLauncherPoints.size) {
                currentLaunchPointIndex = 0;
            }
        } else {
            currentLaunchPointIndex -= 1

            if (currentLaunchPointIndex < 0) {
                currentLaunchPointIndex = LauncherPoint.blueLauncherPoints.size - 1;
            }
        }

        currentLaunchPoint = LauncherPoint.blueLauncherPoints[currentLaunchPointIndex]
        robot.launcher.baseRPM = currentLaunchPoint.launcherRPM
    }
}
