package org.firstinspires.ftc.teamcode.Competition_Code.Tele.OpModes

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.Competition_Code.Actions.InterleagueActions
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
import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.Angles
import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.goalAngle
import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.goalAngleAdjusted
import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.launcherSpeed
import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.launcherSpeedAdjusted
import kotlin.math.PI

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

    // Variables
    val packet: TelemetryPacket
    var runningActions: ArrayList<Action>
    var shootingActions: ArrayList<Action>

    var robot: InterleagueActions
    var drive: Drivetrain
    var localizer: Localizer
    var driveOverride: DrivetrainOverride
    var driveOverrideSafetyTimer: Long
    var driveOffset: Double
    var turnOffset: Double
    var rpmScaling: Boolean

    val startIntake: Button
    val reverseIntake: Button
    val shoot: Button
    val increaseRPM: Button
    val decreaseRPM: Button
    val increaseOffset: Button
    val decreaseOffset: Button
    val slowMode: Button
    val resetOdo: Button
    val resetRPM: Button
    val driveTo: Button
    val rotateTo: Button



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
        turnOffset = if(AutoGlobals.FarAuto) {
            when (color) {
                AllianceColor.Blue -> 0.03
                AllianceColor.Red -> 0.05
            }
        } else {
            when (color) {
                AllianceColor.Blue -> 0.02
                AllianceColor.Red -> 0.03
            }
        }


        packet = TelemetryPacket()
        runningActions = ArrayList<Action>()
        shootingActions = ArrayList<Action>()

        startIntake = Button()
        reverseIntake = Button()
        shoot = Button()
        increaseRPM = Button()
        decreaseRPM = Button()
        increaseOffset = Button()
        decreaseOffset = Button()
        slowMode = Button()
        resetOdo = Button()
        resetRPM = Button()
        driveTo = Button()
        rotateTo = Button()

        robot = InterleagueActions(hardwareMap, telemetry)

        drive = Drivetrain(hardwareMap, color)
        localizer = Localizer(hardwareMap, TeleGlobals.currentPosition)

        telemetry.addData("Robot at zero:",  Localizer.pose)
        localizer.update()
        telemetry.addData("Robot at original position:",  Localizer.pose)

        driveOverride = DrivetrainOverride()

        driveOverrideSafetyTimer = 0L

        if(!AutoGlobals.FarAuto){
            robot.launcher.change = -50
        }

        robot.holder.state = Servo.State.STOP1
        telemetry.update()
        rpmScaling = true

    }

    fun start() {
        localizer.update()
        localizer.transferToTele()

        telemetry.clear()
        robot.launcher.start()

        hasStarted = true
    }

    fun update() {
        require(hasStarted) { "BaseTele has not been started!" }

        handleInputs()
        updateActions()
        updateDrive()
        updateRobot()
        updateTelemetry()
    }

    fun handleInputs(){
        val intaking = robot.intake.state == Intake.State.INTAKING
        val reversing = robot.intake.state == Intake.State.REVERSE

        //Shoot sequence
        if (shoot.pressed(gamepad1.right_trigger >= 0.5) && shootingActions.isEmpty()) {
            // Add the shooting action to the list of running actions
            shootingActions.add(robot.ShootTele())
        }

        //intake toggle
        if(startIntake.pressed(gamepad1.left_trigger >= 0.5)){
            runningActions.add(
                if (!intaking) robot.StartIntake else robot.StopIntake
            )
        }

        //reverse intake toggle
        if (reverseIntake.pressed(gamepad1.dpad_down)) {
            runningActions.add(
                if (!reversing) robot.ReverseIntake else robot.StopIntake
            )
        }

        //shot power adjustments
        if (increaseRPM.pressed(gamepad1.right_bumper)) {
            robot.launcher.change += 50
        }
        if (decreaseRPM.pressed(gamepad1.left_bumper)) {
            robot.launcher.change -= 50
        }

        //adjust launch angle
        if(increaseOffset.pressed(gamepad1.dpad_right)){
            turnOffset += 0.05
        }
        if(decreaseOffset.pressed(gamepad1.dpad_left)){
            turnOffset -= 0.05
        }

        //drivetrain override
        if (driveTo.pressed(gamepad1.y)) {
            driveOverride.beginOverriding(currentLaunchPoint.pose)
        }

        //rotational overide
        if (rotateTo.pressed(gamepad1.right_stick_button)) {
            driveOverride.stopOverriding()

            driveShouldRotate = !driveShouldRotate
        }

        //reset odometry
        if (resetOdo.pressed(gamepad1.a)) {
            localizer.resetOdo()
        }

        //slow mode
        if (slowMode.pressed(gamepad1.b)) {
            drive.slowToggle()
        }

        //reset rpm for launcher
        if(resetRPM.pressed(gamepad1.x)){
            rpmScaling = !rpmScaling
        }
    }
    fun updateActions(){
        runningActions.removeIf {
            it.preview(packet.fieldOverlay())
            !it.run(packet)
        }

        val wasShooting = shootingActions.isNotEmpty()

        shootingActions.removeIf {
            it.preview(packet.fieldOverlay())
            !it.run(packet)
        }

        val doneShooting = wasShooting && shootingActions.isEmpty()

        if (doneShooting){
            driveShouldRotate = false
        }
    }
    fun updateDrive(){
        localizer.update()
        TeleGlobals.currentPosition = Localizer.pose

        // drivetrain overrides
        if (driveOverride.shouldOverrideInput()) {
            if (driveOverride.safetyMeasures(gamepad1)) {
                driveOverrideSafetyTimer = System.currentTimeMillis()
            }

            driveOverride.update(drive)
        } else if (driveShouldRotate) {
            val launchAngle = turnOffset + goalAngleAdjusted(Localizer.velocity.x, Localizer.velocity.y,
                Localizer.pose.x, Localizer.pose.y, allianceColor)
            driveOverride.rotate(
                drive, launchAngle, gamepad1, allianceColor
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
    }

    fun updateRobot(){
        robot.launcher.baseRPM =
            if (rpmScaling)
                launcherSpeedAdjusted(Localizer.velocity.x, Localizer.velocity.y, Localizer.pose.x,
                    Localizer.pose.y, allianceColor)
            else
                2500.0
        robot.update()
    }
    fun updateTelemetry(){
        val overrideTimeLeft = System.currentTimeMillis() - driveOverrideSafetyTimer
        if (overrideTimeLeft < 5000) {
            telemetry.addData("Drive train override safety was tripped!", overrideTimeLeft)
        }

        telemetry.addData("Launcher rpm goal", robot.launcher.goalRPM)
        telemetry.addData("Launcher at RPM?", robot.launcher.atTargetRPM(robot.launcher.goalRPM, 100.0))
        telemetry.addData("goal angle",  turnOffset + goalAngleAdjusted(Localizer.velocity.x, Localizer.velocity.y,
            Localizer.pose.x, Localizer.pose.y, allianceColor))
        telemetry.addData("original angle",  turnOffset + PI + goalAngle(Localizer.pose.x, Localizer.pose.y, allianceColor))


        telemetry.addData("Current Pose", Localizer.pose.toString())
        telemetry.addData("Current Velo", Localizer.velocity.toString())

        telemetry.update()
    }

    class Button(){
        private var pressedLast = false

        fun pressed(condition: Boolean): Boolean {
            val result = condition && !pressedLast
            pressedLast = condition
            return result
        }

        fun released(condition: Boolean): Boolean {
            val result = !condition && pressedLast
            pressedLast = condition
            return result
        }

        fun held(condition: Boolean): Boolean {
            pressedLast = condition
            return condition
        }
    }

}