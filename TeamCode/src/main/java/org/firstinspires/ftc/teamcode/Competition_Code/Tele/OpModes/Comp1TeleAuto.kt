package org.firstinspires.ftc.teamcode.Competition_Code.Tele.OpModes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime

import org.firstinspires.ftc.teamcode.Competition_Code.Actions.Comp1Actions
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Drivetrain
import org.firstinspires.ftc.teamcode.Competition_Code.PinpointLocalizer.Localizer
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.DrivetrainOverride
import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.Poses

//todo test after getting wheels in right directions
@TeleOp(name = "Comp1TeleAuto", group = "Linear OpMode")
class Comp1TeleAuto : LinearOpMode() {

    override fun runOpMode() {
        val dash: FtcDashboard = FtcDashboard.getInstance()
        val packet = TelemetryPacket()
        var runningActions = ArrayList<Action>()

        var balls = 0             // Tracks the next ball to intake
        val buttonDebounce = 500 // ms minimum between button presses
        val buttonTimer = ElapsedTime()
        var intaking = false

        val robot = Comp1Actions(hardwareMap)
        val timer = ElapsedTime()

        val drive = Drivetrain(hardwareMap)
        val localizer = Localizer(hardwareMap, Poses(0.0, 0.0, 0.0))

        val driveOverride = DrivetrainOverride()

        /**
         * This is only used for telemetry, nothing more
         */
        val driveOverrideSafetyTimer = 0.0f

        while (opModeInInit()) timer.reset()

        while (opModeIsActive()) {

            // SHOOTING: A button triggers full ShootBalls sequence
            if (gamepad2.right_trigger >= 0.5) {
                runningActions.add(robot.ShootBalls)
                balls = 0  // Reset intake counter after shooting
            }

            if (gamepad2.left_trigger >= 0.5 && buttonTimer.milliseconds() >= buttonDebounce) {
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
                        if (robot.ball1.checkForRecognition()){
                            runningActions.add(robot.HoldBall)
                            balls += 1
                        }
                    }
                    1 -> {
                        if (robot.ball2.checkForRecognition()){
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

            if (driveOverride.shouldOverrideInput()) {
                if (driveOverride.safetyMeasures(gamepad2)) {
                    System.currentTimeMillis()
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

            packet.put("Running Actions", runningActions.size)
            packet.put("Balls", balls)

            val overrideTimeLeft = System.currentTimeMillis() - driveOverrideSafetyTimer
            if (overrideTimeLeft < 5000) {
                packet.put("Drive train override safety was tripped!", overrideTimeLeft)
            }

            dash.sendTelemetryPacket(packet)

            if(gamepad1.a){
                localizer.resetHeading()
            }

            timer.reset()
        }
    }
}
