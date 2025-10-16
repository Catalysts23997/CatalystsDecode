package org.firstinspires.ftc.teamcode.Competition_Code.Tele.OpModes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime

import org.firstinspires.ftc.teamcode.Competition_Code.Actions.Comp1Actions
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Drivetrain
import org.firstinspires.ftc.teamcode.Competition_Code.PinpointLocalizer.Localizer
import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.Poses

//todo test after getting wheels in right directions
@TeleOp(name = "Comp1Tele", group = "Linear OpMode")
class Comp1Tele : LinearOpMode() {

    override fun runOpMode() {
        val dash: FtcDashboard = FtcDashboard.getInstance()
        val packet = TelemetryPacket()
        var runningActions = ArrayList<Action>()

        val robot = Comp1Actions(hardwareMap)
        val timer = ElapsedTime()

        val drive = Drivetrain(hardwareMap)
        val localizer = Localizer(hardwareMap, Poses(0.0, 0.0, 0.0))

        while (opModeInInit()) timer.reset()

        while (opModeIsActive()) {

            if(gamepad2.a){
                runningActions.add(
                    SequentialAction(
                        robot.Shoot,
                        if (robot.stop.checkForRecognition()){
                            robot.Cycle
                        } else{
                            SleepAction(0.0)
                        }
                    )
                )
            }
            if(gamepad2.b){
                runningActions.add(
                    robot.Cycle
                )
            }
            if(gamepad2.x){
                runningActions.add(
                    robot.StartIntake
                )
            }
            if(gamepad2.y){
                runningActions.add(
                    robot.StopIntake
                )
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
            dash.sendTelemetryPacket(packet)

            //update subsystems
            localizer.update()
            robot.update()
            drive.update(
                arrayListOf(
                    gamepad1.left_stick_x,
                    -gamepad1.left_stick_y,
                    gamepad1.right_stick_x
                )
            )

            if(gamepad1.a){
                localizer.resetHeading()
            }

            timer.reset()
        }
    }
}
