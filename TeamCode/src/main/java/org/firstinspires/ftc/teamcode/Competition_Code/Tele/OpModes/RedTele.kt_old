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
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Servo
import org.firstinspires.ftc.teamcode.Competition_Code.Tele.TeleGlobals

@TeleOp(name = "RedTele", group = "Linear OpMode")
class RedTele : LinearOpMode() {

    // get the current launcher point
    var currentLaunchPointIndex = LauncherPoint.getPriorityPoint(LauncherPoint.redLauncherPoints)
    var currentLaunchPoint: LauncherPoint = LauncherPoint.redLauncherPoints[currentLaunchPointIndex]
    lateinit var robot: Comp2Actions

    var driveShouldRotate = false
    var driveRotateResistance = 0.6f;
    var driveRotateSoftLock = false

    override fun runOpMode() {

        if(AutoGlobals.AutonomousRan) {
            TeleGlobals.currentPosition = AutoGlobals.locationOfRobot!!
        } else {
            TeleGlobals.currentPosition = AutoPoints.StartRed.pose
        }
        telemetry.addData("Robot at position ",  TeleGlobals.currentPosition)

        val dash: FtcDashboard = FtcDashboard.getInstance()
        telemetry = dash.telemetry

        val packet = TelemetryPacket()
        var runningActions = ArrayList<Action>()

        var balls = 0             // Tracks the next ball to intake
        val buttonDebounce = 200 // ms minimum between button presses
        val buttonTimer = ElapsedTime()

        var intaking = false
        var reversing = false

        robot = Comp2Actions(hardwareMap, telemetry)

        val drive = Drivetrain(hardwareMap, Drivetrain.Alliance.Red)
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

            // SHOOTING: A button triggers full Shoot3Balls sequence
            if (gamepad1.right_trigger >= 0.5 && buttonTimer.milliseconds() >= buttonDebounce) {

                runningActions.add(robot.ShootThrough())

                balls = 0  // Reset intake counter after shooting
                buttonTimer.reset()
            }

            if (gamepad1.dpad_down && buttonTimer.milliseconds() >= buttonDebounce) {
                if(!reversing){
                    runningActions.add(robot.ReverseIntake)
                    reversing = true
                    intaking = false
                }
                else{
                    runningActions.add(robot.StopIntake)
                    reversing = false
                    intaking = false
                }

                buttonTimer.reset()
            }

            if (gamepad1.left_trigger >= 0.5 && buttonTimer.milliseconds() >= buttonDebounce) {
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

            //testing

            if (gamepad2.right_bumper && buttonTimer.milliseconds() >= buttonDebounce){
                robot.holder.launchpos +=0.01
                buttonTimer.reset()

            }
            if (gamepad2.left_bumper && buttonTimer.milliseconds() >= buttonDebounce) {
                robot.holder.launchpos -= 0.01
                buttonTimer.reset()

            }

            if (gamepad1.right_bumper && buttonTimer.milliseconds() >= buttonDebounce){
                robot.launcher.change += 100
                buttonTimer.reset()
            }
            if (gamepad1.left_bumper && buttonTimer.milliseconds() >= buttonDebounce){
                robot.launcher.change -= 100
                buttonTimer.reset()
            }

            if (intaking) {
                when (balls){
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

            TeleGlobals.currentPosition = Localizer.pose

            //updatePID subsystems
            if(gamepad1.a){
                localizer.resetOdo()
            }


            localizer.update()
            robot.update()

            // BEGIN LAUNCHER DRIVETRAIN CODE

            if (gamepad1.dpad_right && buttonTimer.milliseconds() >= buttonDebounce) {
                cycleLauncherPoint(true)
                buttonTimer.reset()
            } else if (gamepad1.dpad_left && buttonTimer.milliseconds() >= buttonDebounce) {
                cycleLauncherPoint(false)
                buttonTimer.reset()
            }

            if (gamepad1.dpad_up && buttonTimer.milliseconds() >= buttonDebounce){
                currentLaunchPointIndex = 0
                currentLaunchPoint = LauncherPoint.redLauncherPoints[currentLaunchPointIndex]
                robot.launcher.baseRPM = currentLaunchPoint.launcherRPM
                buttonTimer.reset()
            }

            if (gamepad1.y && buttonTimer.milliseconds() >= buttonDebounce) {
                driveOverride.beginOverriding(currentLaunchPoint.pose)
                buttonTimer.reset()

                // TODO:
                // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
                // @              should this button have debounce?              @
                // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
            }

            if (/* the control to toggle the rotation*/ false && buttonTimer.milliseconds() >= buttonDebounce) {
                // when changing modes in the drivetrain override,
                // just stop overriding
                driveOverride.stopOverriding()

                driveShouldRotate = !driveShouldRotate
                buttonTimer.reset()
            }

            if (/* the control to toggle soft lock */ false && buttonTimer.milliseconds() >= buttonDebounce) {
                driveRotateSoftLock = !driveRotateSoftLock
                buttonTimer.reset()
            }

            if (/* the control to increase the resistance */ false && buttonTimer.milliseconds() >= buttonDebounce) {
                driveRotateResistance += 0.1f
                buttonTimer.reset()
            } else if (/* the control to decrease the resistance */ false && buttonTimer.milliseconds() >= buttonDebounce) {
                driveRotateResistance -= 0.1f
                buttonTimer.reset()
            }

            // END LAUNCHER DRIVETRAIN CODE

            if(gamepad1.b){
                driveOverride.beginOverriding(AutoPoints.EndgameRed.pose)
            }

            if (driveOverride.shouldOverrideInput()) {
                if (driveOverride.safetyMeasures(gamepad1)) {
                    driveOverrideSafetyTimer = System.currentTimeMillis()
                }

                driveOverride.update(drive)
            } else {
                drive.update(
                    arrayListOf(
                        gamepad1.left_stick_x,
                        -gamepad1.left_stick_y,
                        gamepad1.right_stick_x
                    )
                )

                if (driveShouldRotate) {
                    driveOverride.rotate(
                        drive, 0.0, gamepad1
                    )
                }
            }

            val overrideTimeLeft = System.currentTimeMillis() - driveOverrideSafetyTimer
            if (overrideTimeLeft < 5000) {
                telemetry.addData("Drive train override safety was tripped!", overrideTimeLeft)
            }

            telemetry.addData("Launcher rpm goal", robot.launcher.goalRPM)
            telemetry.addData("Launcher at RPM?", robot.launcher.atTargetRPM(robot.launcher.goalRPM, 100.0))
            telemetry.addData("servopos", robot.holder.launchpos)
            telemetry.addData("Current Pose", Localizer.pose.toString())

            telemetry.update()
        }
    }

    fun cycleLauncherPoint(forwards: Boolean) {
        if (forwards) {
            currentLaunchPointIndex += 1

            if (currentLaunchPointIndex >= LauncherPoint.redLauncherPoints.size) {
                currentLaunchPointIndex = 0;
            }
        } else {
            currentLaunchPointIndex -= 1

            if (currentLaunchPointIndex < 0) {
                currentLaunchPointIndex = LauncherPoint.redLauncherPoints.size - 1;
            }
        }

        currentLaunchPoint = LauncherPoint.redLauncherPoints[currentLaunchPointIndex]
        robot.launcher.baseRPM = currentLaunchPoint.launcherRPM
    }
}
