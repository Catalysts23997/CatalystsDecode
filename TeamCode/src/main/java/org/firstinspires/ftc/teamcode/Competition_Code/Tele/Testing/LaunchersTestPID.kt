package org.firstinspires.ftc.teamcode.Competition_Code.Tele.Extra

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Launcher
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Servo
import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.PIDParams

@Config
@TeleOp(group = "Linear OpMode", name = "LaunchersTestPID")
class
LaunchersTestPID: LinearOpMode() {

    companion object{

        @JvmField var p1 = 0.0035
        @JvmField var i1 = 0.0
        @JvmField var d1 = 0.0001
        @JvmField var f1 = 1/Launcher.leftMaxRPM

        @JvmField var p2 = 0.0035
        @JvmField var i2 = 0.0
        @JvmField var d2 = 0.0001
        @JvmField var f2 = 1/ Launcher.rightMaxRPM

        @JvmField var targetRpm = 3538.0
    }

    override fun runOpMode() {
        telemetry = FtcDashboard.getInstance().telemetry

        var leftParams: PIDParams
        var rightParams: PIDParams
        val launcher = Launcher(hardwareMap)


        val kicker = Servo(hardwareMap, "kicker")
        val holder = Servo(hardwareMap, "holder")

        launcher.start()

        waitForStart()
        while (opModeIsActive()){
            kicker.update()
            holder.update()
            leftParams = PIDParams(p1,i1,d1,f1)
            rightParams = PIDParams(p2,i2,d2,f2)
            launcher.setLeftPidParams(leftParams)
            launcher.setRightPidParams(rightParams)

            launcher.baseRPM = targetRpm
            launcher.update()

            val packet = TelemetryPacket()

            packet.put("goalRPM", targetRpm)
            packet.put("leftRPM", launcher.leftRpm)
            packet.put("rightRPM", launcher.rightRpm)
            packet.put("leftPower", launcher.leftPower)
            packet.put("rightPower", launcher.rightPower)


            FtcDashboard.getInstance().sendTelemetryPacket(packet)


        }
    }

}