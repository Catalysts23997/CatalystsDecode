package org.firstinspires.ftc.teamcode.Competition_Code.Tele.Extra

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Launcher
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Servo
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.SingleLauncher
import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.PIDParams

@Config
@TeleOp(group = "Linear OpMode", name = "SingleLaunchersTestPID")
class
SingleLauncherTestPID: LinearOpMode() {

    companion object{

        @JvmField var p1 = 0.000
        @JvmField var i1 = 0.0
        @JvmField var d1 = 0.000
        @JvmField var f1 = 1.0/5200


        @JvmField var targetRpm = 0.0
    }

    override fun runOpMode() {
        telemetry = FtcDashboard.getInstance().telemetry

        var leftParams: PIDParams

        val launcher = SingleLauncher(hardwareMap)

        launcher.start()

        waitForStart()
        while (opModeIsActive()){

            leftParams = PIDParams(p1,i1,d1,f1)

            launcher.setPidParams(leftParams)

            launcher.baseRPM = targetRpm
            launcher.update()

            val packet = TelemetryPacket()

            packet.put("goalRPM", targetRpm)
            packet.put("leftRPM", launcher.rpm)
            packet.put("leftPower", launcher.power)


            FtcDashboard.getInstance().sendTelemetryPacket(packet)


        }
    }

}