package org.firstinspires.ftc.teamcode.Competition_Code.Tele.Testing

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Competition_Code.Actions.Comp1Actions
import org.firstinspires.ftc.teamcode.Competition_Code.AllianceColor
import org.firstinspires.ftc.teamcode.Competition_Code.PinpointLocalizer.Localizer
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Drivetrain
import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.Poses

@Disabled
@TeleOp(name = "NoAutoTele", group = "Linear OpMode")
class NoAutoTele : LinearOpMode() {

    override fun runOpMode() {
        val dash: FtcDashboard = FtcDashboard.getInstance()
        val packet = TelemetryPacket()
        var runningActions = ArrayList<Action>()

        var balls = 0             // Tracks the next ball to intake
        val buttonDebounce = 500 // ms minimum between button presses
        val buttonTimer = ElapsedTime()
        var intaking = false

        val robot = Comp1Actions(hardwareMap, telemetry)
        val timer = ElapsedTime()

        val drive = Drivetrain(hardwareMap, AllianceColor.Red)
        val localizer = Localizer(hardwareMap, Poses(0.0, 0.0, 0.0))



        while (opModeInInit()) timer.reset()

        while (opModeIsActive()) {

            // SHOOTING: A button triggers full Shoot3Balls sequence
            if (gamepad2.right_trigger >= 0.5) {
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
                        if (robot.ball1.isGreen() || robot.ball1.isPurple()){
                            runningActions.add(robot.HoldBall)
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

            //updatePID subsystems
            localizer.update()
            robot.update()


            drive.update(
                arrayListOf(
                    gamepad1.left_stick_x,
                    -gamepad1.left_stick_y,
                    gamepad1.right_stick_x
                )
            )

            packet.put("Running Actions", runningActions.size)
            packet.put("BallsIntake", balls)

            dash.sendTelemetryPacket(packet)

            if(gamepad1.a){
                localizer.resetOdo()
            }

            timer.reset()
        }
    }
}