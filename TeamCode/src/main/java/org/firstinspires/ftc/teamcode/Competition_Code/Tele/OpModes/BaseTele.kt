package org.firstinspires.ftc.teamcode.Competition_Code.Tele.OpModes

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.Competition_Code.Actions.Comp1Actions
import org.firstinspires.ftc.teamcode.Competition_Code.Auto.AutoGlobals
import org.firstinspires.ftc.teamcode.Competition_Code.Auto.AutoPoints
import org.firstinspires.ftc.teamcode.Competition_Code.PinpointLocalizer.Localizer
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Drivetrain
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.DrivetrainOverride
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Servo
import org.firstinspires.ftc.teamcode.Competition_Code.Tele.TeleGlobals
import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.Poses

enum class TeleColor {
    Blue,
    Red
}

/**
 * This class is **NOT** an OpMode, it is used to store common code that
 * both the red and blue tele-op should share.
 *
 * ## Notes
 * This code is written like a subsystem.
 */
// OpMode stores the current opmode that this code is attached to.
class BaseTele(opmode: LinearOpMode, color: TeleColor) {

    var hasStarted: Boolean = false

    // Define a few variables for convenience
    var telemetry: Telemetry = opmode.telemetry;
    var hardwareMap: HardwareMap = opmode.hardwareMap;
    var gamepad1: Gamepad = opmode.gamepad1

    // Variables
    val packet: TelemetryPacket
    var runningActions: ArrayList<Action>
    var balls: Int
    var buttonDebounce: Int
    var buttonTimer: ElapsedTime
    var intaking: Boolean
    var reversing: Boolean
    var robot: Comp1Actions
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

    /**
     * Sets the default state and configures variables.
     */
    init {
        // Get the location of the robot
        if (AutoGlobals.AutonomousRan) {
            TeleGlobals.currentPosition = AutoGlobals.locationOfRobot!!
        } else {
            TeleGlobals.currentPosition = AutoPoints.StartBlue.pose
        }

        telemetry.addData("Robot at position ",  TeleGlobals.currentPosition)

        // Setup variables
        driveOffset = when (color) {
            TeleColor.Blue -> -Math.PI / 2
            TeleColor.Red -> Math.PI / 2
        }

        packet = TelemetryPacket()
        runningActions = ArrayList<Action>()

        balls = 0             // Tracks the next ball to intake
        // TODO: ensure that this is the correct value
        buttonDebounce = 100 // ms minimum between button presses
        buttonTimer = ElapsedTime()

        intaking = false
        reversing = false

        robot = Comp1Actions(hardwareMap, telemetry)

        drive = Drivetrain(hardwareMap)
        localizer = Localizer(hardwareMap, TeleGlobals.currentPosition)

        telemetry.addData("Robot at zero:",  Localizer.pose)
        localizer.update()
        telemetry.addData("Robot at original position:",  Localizer.pose)

        driveOverride = DrivetrainOverride()


        /**
         * This is only used for telemetry, nothing more
         */
        driveOverrideSafetyTimer = 0L

        robot.holder.state = Servo.State.STOP
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

        // SHOOTING: A button triggers full Shoot3Balls sequence
        if (gamepad1.left_trigger >= 0.5 && buttonTimer.milliseconds() >= buttonDebounce) {
            // Add the shooting action to the list of running actions
            runningActions.add(robot.ShootThrough())

            // After we shoot 3 calls, we will have none remaining
            balls = 0  // Reset intake counter after shooting
            buttonTimer.reset()
        }

        val triggerPressed = gamepad1.right_trigger > 0.5
        if (triggerPressed && !lastTriggerPressed && buttonTimer.milliseconds() > buttonDebounce) {
            // We want to shoot 1 ball
            shotsRequested += 1
            buttonTimer.reset()
        }

        // Make sure that we only run the IF statement above once, even if
        // the trigger is held for longer
        lastTriggerPressed = triggerPressed

        // We want to ensure the shoot ball action gets added if we want to shoot
        if (!shooting && shotsRequested > 0) {
            runningActions.add(robot.ShootFirstBall())
            shotTimer.reset()
            shooting = true
            firstShot = true
        }

        // Shoot the balls
        if (shooting) {
            val requiredTime = if (firstShot) 3.0 else 2.0

            if (shotTimer.seconds() >= requiredTime) {
                shotsRequested -= 1

                if (shotsRequested == 0) {
                    runningActions.add(robot.StopShooter)
                    shooting = false
                } else {
                    runningActions.add(robot.ShootBall())
                    firstShot = false
                    shotTimer.reset()
                }
            }
        }


        // Handle the intake
        if (gamepad1.dpad_down && buttonTimer.milliseconds() >= buttonDebounce) {
            if (!reversing) {
                runningActions.add(robot.ReverseIntake)
                reversing = true
                intaking = false
            } else {
                runningActions.add(robot.StopIntake)
                reversing = false
                intaking = false
            }

            buttonTimer.reset()
        }

        if (gamepad1.dpad_up && buttonTimer.milliseconds() >= buttonDebounce) {
            if(!intaking){
                runningActions.add(robot.StartIntake)
                intaking = true
                reversing = false
            }
            else{
                runningActions.add(robot.StopIntake)
                intaking = false
                reversing = false
            }

            buttonTimer.reset()
        }

        // Detect if we have picked up any balls
        if (intaking) {
            when (balls) {
                0 -> {
                    if (robot.ball1.isGreen() || robot.ball1.isPurple()){
                        balls += 1
                    }
                }
                1 -> {
                    if (robot.ball2.isGreen() || robot.ball2.isPurple()){
                        balls += 1
                    }
                }
            }
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

        // Update our current position
        TeleGlobals.currentPosition = Localizer.pose

        // Update the localizer and robot
        localizer.update()
        robot.update()

        // Drivetrain override
        if(gamepad1.y){
            driveOverride.beginOverriding(AutoPoints.LaunchBlue.pose)
        }
        if(gamepad1.b){
            driveOverride.beginOverriding(Poses(34.0, -38.0, 0.0))
        }

        // Update the drivetrain
        if (driveOverride.shouldOverrideInput()) {
            // The DrivetrainOverride code is controlling our movement
            if (driveOverride.safetyMeasures(gamepad1)) {
                driveOverrideSafetyTimer = System.currentTimeMillis()
            }

            driveOverride.update(drive)
        } else {
            // The gamepad is controlling our movement
            drive.update(
                arrayListOf(
                    gamepad1.left_stick_x,
                    -gamepad1.left_stick_y,
                    gamepad1.right_stick_x
                ), driveOffset
            )
        }

        // Add some telemetry
        val overrideTimeLeft = System.currentTimeMillis() - driveOverrideSafetyTimer
        if (overrideTimeLeft < 5000) {
            telemetry.addData("Drive train override safety was tripped!", overrideTimeLeft)
        }

        telemetry.addData("Current Pose", Localizer.pose.toString())

        telemetry.update()
    }

}