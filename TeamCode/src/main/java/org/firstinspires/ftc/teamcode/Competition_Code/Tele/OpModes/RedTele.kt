package org.firstinspires.ftc.teamcode.Competition_Code.Tele.OpModes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Competition_Code.Actions.Comp1Actions
import org.firstinspires.ftc.teamcode.Competition_Code.Auto.OpModes.BlueAuto

import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Drivetrain
import org.firstinspires.ftc.teamcode.Competition_Code.PinpointLocalizer.Localizer
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.DrivetrainOverride
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Servo
import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.Poses

//todo test after getting wheels in right directions
@TeleOp(name = "RedTele", group = "Linear OpMode")
class RedTele : LinearOpMode() {

    override fun runOpMode() {

        if(BlueAuto.endPos == Poses(0.0,0.0,0.0))    {
            BlueAuto.endPos = Poses(39.0,63.0,0.0)
        }

        val dash: FtcDashboard = FtcDashboard.getInstance()
        val packet = TelemetryPacket()
        var runningActions = ArrayList<Action>()

        var balls = 0             // Tracks the next ball to intake
        val buttonDebounce = 200 // ms minimum between button presses
        val buttonTimer = ElapsedTime()
        var intaking = false

        val robot = Comp1Actions(hardwareMap, telemetry)
        val timer = ElapsedTime()

        val drive = Drivetrain(hardwareMap)
        val localizer = Localizer(hardwareMap, BlueAuto.endPos)

        val driveOverride = DrivetrainOverride()

        /**
         * This is only used for telemetry, nothing more
         */
        var driveOverrideSafetyTimer: Long = 0L

        while (opModeInInit()) timer.reset()
        robot.holder.state = Servo.State.STOP

        while (opModeIsActive()) {


            // SHOOTING: A button triggers full Shoot3Balls sequence
            if (gamepad1.right_trigger >= 0.5 && buttonTimer.milliseconds() >= buttonDebounce) {

                runningActions.add(robot.ShootThrough())

                balls = 0  // Reset intake counter after shooting
                buttonTimer.reset()
            }
            if (gamepad1.right_bumper && buttonTimer.milliseconds() >= buttonDebounce) {

                runningActions.add(robot.Shoot3Balls())

                balls = 0  // Reset intake counter after shooting
                buttonTimer.reset()
            }

            if (gamepad1.left_trigger >= 0.5 && buttonTimer.milliseconds() >= buttonDebounce) {
                if(!intaking){
                    runningActions.add(robot.StartIntake)
                    intaking = true
                }
                else{
                    runningActions.add(robot.StopIntake)
                    intaking = false
                }
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

            // update running actions
            val newActions = ArrayList<Action>()
            runningActions.forEach {
                it.preview(packet.fieldOverlay())
                if (it.run(packet)) {
                    newActions.add(it)
                }
            }
            runningActions = newActions

            //update subsystems
            localizer.update()
            robot.update()

            if(gamepad1.y){
                driveOverride.beginOverriding(Poses(10.0, 15.0, -11*Math.PI/16))
            }
            if(gamepad1.b){
                driveOverride.beginOverriding(Poses(-34.0, -38.0, 0.0))
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
                    ), Math.PI/2
                )
            }

            telemetry.addData("Running Actions", runningActions.size)
            telemetry.addData("BallsIntake", balls)

            val overrideTimeLeft = System.currentTimeMillis() - driveOverrideSafetyTimer
            if (overrideTimeLeft < 5000) {
                telemetry.addData("Drive train override safety was tripped!", overrideTimeLeft)
            }

            dash.sendTelemetryPacket(packet)

            if(gamepad1.a){
                localizer.resetHeading()
            }


            telemetry.addData("Is intaking?", intaking)

            telemetry.addData("x", gamepad1.left_stick_x)
            telemetry.addData("y", gamepad1.left_stick_y)

            telemetry.addData("Heading", Localizer.pose.heading)
            telemetry.addData("X position", Localizer.pose.x)
            telemetry.addData("Y position", Localizer.pose.y)
            telemetry.update()

            timer.reset()
        }
    }
}
