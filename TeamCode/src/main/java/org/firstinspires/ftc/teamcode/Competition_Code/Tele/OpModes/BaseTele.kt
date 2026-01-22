// NOTE: This file was created from the old BlueTele.kt file.
// TODO: We need to verify that this code is the same as the old RedTele.kt file.

package org.firstinspires.ftc.teamcode.Competition_Code.Tele.OpModes

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.Competition_Code.Actions.Comp2Actions
import org.firstinspires.ftc.teamcode.Competition_Code.AllianceColor
import org.firstinspires.ftc.teamcode.Competition_Code.Auto.AutoGlobals
import org.firstinspires.ftc.teamcode.Competition_Code.Auto.AutoPoints
import org.firstinspires.ftc.teamcode.Competition_Code.PinpointLocalizer.Localizer
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Drivetrain
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.DrivetrainOverride
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Intake
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.LauncherPoint
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Servo
import org.firstinspires.ftc.teamcode.Competition_Code.Tele.TeleGlobals
import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.goalAngle

/**
 * This class is **NOT** an OpMode, it is used to store common code that
 * both the red and blue tele-op should share.
 *
 * ## Notes
 * This code is written like a subsystem.
 */
// OpMode stores the current opmode that this code is attached to.
class BaseTele(opmode: LinearOpMode, color: AllianceColor) {

    var hasStarted: Boolean = false

    // Define a few variables for convenience
    var telemetry: Telemetry = opmode.telemetry
    var hardwareMap: HardwareMap = opmode.hardwareMap
    var gamepad1: Gamepad = opmode.gamepad1
    var gamepad2: Gamepad = opmode.gamepad2

    // Variables
    val packet: TelemetryPacket
    var runningActions: ArrayList<Action>
    var buttonDebounce: Int
    var buttonTimer: ElapsedTime
    var robot: Comp2Actions
    var drive: Drivetrain
    var localizer: Localizer
    var driveOverride: DrivetrainOverride
    var driveOverrideSafetyTimer: Long
    var shotsRequested: Int
    var firstShot: Boolean
    var shooting: Boolean
    val shotTimer: ElapsedTime
    var lastTriggerPressed: Boolean
    var driveOffset: Double

    // get the current launcher point
    var launcherPoints: Array<LauncherPoint> = LauncherPoint.getLauncherPoints(color)
    var currentLaunchPointIndex = LauncherPoint.getPriorityPoint(color)
    var currentLaunchPoint: LauncherPoint = launcherPoints[currentLaunchPointIndex]

    val allianceColor: AllianceColor = color

    var driveShouldRotate = false

    /**
     * Sets the default state and configures variables.
     */
    init {
        // Get the location of the robot
        if (AutoGlobals.AutonomousRan) {
            TeleGlobals.currentPosition = AutoGlobals.locationOfRobot!!
        } else {
            TeleGlobals.currentPosition = when (color) {
                AllianceColor.Blue -> AutoPoints.StartBlue.pose
                AllianceColor.Red -> AutoPoints.StartRed.pose
            }
        }

        telemetry.addData("Robot at position ",  TeleGlobals.currentPosition)

        // Setup variables
        driveOffset = when (color) {
            AllianceColor.Blue -> -Math.PI / 2
            AllianceColor.Red -> Math.PI / 2
        }

        packet = TelemetryPacket()
        runningActions = ArrayList<Action>()

        // TODO: ensure that this is the correct value
        buttonDebounce = 100 // ms minimum between button presses
        buttonTimer = ElapsedTime()

        robot = Comp2Actions(hardwareMap, telemetry)

        drive = Drivetrain(hardwareMap, color)
        localizer = Localizer(hardwareMap, TeleGlobals.currentPosition)

        telemetry.addData("Robot at zero:",  Localizer.pose)
        localizer.update()
        telemetry.addData("Robot at original position:",  Localizer.pose)

        driveOverride = DrivetrainOverride()

        /**
         * This is only used for telemetry, nothing more
         */
        driveOverrideSafetyTimer = 0L

        robot.holder.state = Servo.State.STOP1
        shotsRequested = 0
        firstShot = false
        shooting = false
        shotTimer = ElapsedTime()
        lastTriggerPressed = false
        telemetry.update()

    }

    /**
     * Setup teleop. Use after `waitForStart()`
     */
    fun start() {
        // Ensure that the localizer is ready for teleop
        localizer.update()
        localizer.transferToTele()

        telemetry.clear()
        buttonTimer.reset()
    }

    /**
     * Tick
     *
     * ## Notes
     * This function **will throw an exception** if `start` is not called.
     */
    fun update() {
        // !!! Safety check
        if (!hasStarted) {
            throw RuntimeException("BaseTele has not been started! Has `start` been called?")
        }

        // Main code

        val intaking = robot.intake.state == Intake.State.INTAKING
        val reversing = robot.intake.state == Intake.State.REVERSE

        if (gamepad1.left_trigger >= 0.5 && buttonTimer.milliseconds() >= buttonDebounce && runningActions.isEmpty()) {
            // Add the shooting action to the list of running actions
            runningActions.add(robot.ShootThrough())
            buttonTimer.reset()
        }

        if (gamepad1.left_trigger >= 0.5 && buttonTimer.milliseconds() >= buttonDebounce) {
            if (!intaking) {
                runningActions.add(robot.StartIntake)
            } else {
                runningActions.add(robot.StopIntake)
            }

            buttonTimer.reset()
        }

        if (gamepad1.right_bumper && buttonTimer.milliseconds() >= buttonDebounce) {
            robot.launcher.change += 100
            buttonTimer.reset()
        }
        if (gamepad1.left_bumper && buttonTimer.milliseconds() >= buttonDebounce) {
            robot.launcher.change -= 100
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

        // BEGIN LAUNCHER DRIVETRAIN CODE
        if (gamepad1.dpad_up && buttonTimer.milliseconds() >= buttonDebounce){
            currentLaunchPointIndex = 0
            currentLaunchPoint = launcherPoints[currentLaunchPointIndex]
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

        if (gamepad1.a) {
            localizer.resetOdo()
        }

        if (gamepad1.b) {
            val target = when (allianceColor) {
                AllianceColor.Blue -> AutoPoints.EndgameBlue.pose
                AllianceColor.Red -> AutoPoints.EndgameRed.pose
            }

            driveOverride.beginOverriding(target)
        }

        if (gamepad1.right_stick_button && buttonTimer.milliseconds() >= buttonDebounce) {
            // when changing modes in the drivetrain override,
            // just stop overriding
            driveOverride.stopOverriding()

            driveShouldRotate = !driveShouldRotate
            buttonTimer.reset()
        }

        // END LAUNCHER DRIVETRAIN CODE

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

        // drivetrain overrides
        if (driveOverride.shouldOverrideInput()) {
            if (driveOverride.safetyMeasures(gamepad1)) {
                driveOverrideSafetyTimer = System.currentTimeMillis()
            }

            driveOverride.update(drive)
        } else if (driveShouldRotate) {
            val launchAngle = goalAngle(Localizer.pose.x, Localizer.pose.y, allianceColor)
            driveOverride.rotate(
                drive, launchAngle, gamepad1
            )
        } else {
            drive.update(
                arrayListOf(
                    gamepad1.left_stick_x,
                    -gamepad1.left_stick_y,
                    gamepad1.right_stick_x
                )
            )
        }

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

    fun cycleLauncherPoint(forwards: Boolean) {
        if (forwards) {
            currentLaunchPointIndex += 1

            if (currentLaunchPointIndex >= launcherPoints.size) {
                currentLaunchPointIndex = 0
            }
        } else {
            currentLaunchPointIndex -= 1

            if (currentLaunchPointIndex < 0) {
                currentLaunchPointIndex = launcherPoints.size - 1
            }
        }

        currentLaunchPoint = launcherPoints[currentLaunchPointIndex]
        robot.launcher.baseRPM = currentLaunchPoint.launcherRPM
    }

}