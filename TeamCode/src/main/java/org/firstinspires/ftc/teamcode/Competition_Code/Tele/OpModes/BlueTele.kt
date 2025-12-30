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
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Drivetrain
import org.firstinspires.ftc.teamcode.Competition_Code.PinpointLocalizer.Localizer
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.DrivetrainOverride
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Servo
import org.firstinspires.ftc.teamcode.Competition_Code.Tele.TeleGlobals

@TeleOp(name = "BlueTele", group = "Linear OpMode")
class BlueTele : LinearOpMode() {

    override fun runOpMode() {
        val dash: FtcDashboard = FtcDashboard.getInstance()
        telemetry = dash.telemetry

        if(AutoGlobals.AutonomousRan) {
            TeleGlobals.currentPosition = AutoGlobals.locationOfRobot!!
        } else {

            TeleGlobals.currentPosition = AutoPoints.StartBlue.pose
        }

        telemetry.addData("Robot at position ",  TeleGlobals.currentPosition)

        val packet = TelemetryPacket()
        var runningActions = ArrayList<Action>()

        var balls = 0             // Tracks the next ball to intake
        val buttonDebounce = 200 // ms minimum between button presses
        val buttonTimer = ElapsedTime()

        var intaking = false
        var reversing = false

        val robot = Comp2Actions(hardwareMap, telemetry)

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
        var shotsRequested = 0
        var firstShot = false
        var shooting = false
        val shotTimer = ElapsedTime()
        var lastTriggerPressed = false
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

            if (gamepad1.left_trigger >=0.5 && buttonTimer.milliseconds() >= buttonDebounce) {
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

//            if (gamepad1.right_bumper && buttonTimer.milliseconds() >= buttonDebounce){
//                robot.holder.launchpos +=0.01
//                buttonTimer.reset()
//
//            }
//            if (gamepad1.left_bumper && buttonTimer.milliseconds() >= buttonDebounce) {
//                robot.holder.launchpos -= 0.01
//                buttonTimer.reset()
//
//            }

            if (gamepad1.right_bumper && buttonTimer.milliseconds() >= buttonDebounce){
                robot.launchRPM +=100
                buttonTimer.reset()
            }
            if (gamepad1.left_bumper && buttonTimer.milliseconds() >= buttonDebounce){
                robot.launchRPM -=100
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
            if(gamepad1.a ){
                localizer.resetOdo()
            }

            localizer.update()
            robot.update()

            if(gamepad1.y){
                driveOverride.beginOverriding(AutoPoints.LaunchBlue.pose)
            }
            if(gamepad1.b){
                driveOverride.beginOverriding(AutoPoints.EndgameBlue.pose)
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
            }

            val overrideTimeLeft = System.currentTimeMillis() - driveOverrideSafetyTimer
            if (overrideTimeLeft < 5000) {
                telemetry.addData("Drive train override safety was tripped!", overrideTimeLeft)
            }

            telemetry.addData("Launcher power", robot.launchRPM)
            telemetry.addData("servopos", robot.holder.launchpos)

            telemetry.addData("Current Pose", Localizer.pose.toString())

            telemetry.update()
        }


    }
}
