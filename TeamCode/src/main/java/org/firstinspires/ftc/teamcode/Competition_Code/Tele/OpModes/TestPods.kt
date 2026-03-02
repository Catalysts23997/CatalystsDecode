package org.firstinspires.ftc.teamcode.Competition_Code.Tele.OpModes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.teamcode.Competition_Code.AllianceColor
import org.firstinspires.ftc.teamcode.Competition_Code.PinpointLocalizer.Localizer
import org.firstinspires.ftc.teamcode.Competition_Code.Tele.TeleGlobals
import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.Poses

@TeleOp(name = "TestPods", group = "Linear OpMode")
class TestPods : LinearOpMode() {
    override fun runOpMode() {
        // Create the instance of our BaseTele
        val localizer = Localizer(hardwareMap, Poses(10.0,20.0,Math.PI/2))
        sleep(500)


        telemetry.addLine("localizer initialized")
        telemetry.update()


        localizer.odo.resetPosAndIMU()
        sleep(500)



        telemetry.addLine("Pod fully reset")
        telemetry.update()


        //todo try negating the heading value if the opposite of what it should be
        localizer.odo.setPosition(Pose2D(DistanceUnit.INCH,localizer.offset.y, -localizer.offset.x, AngleUnit.RADIANS,-localizer.offset.heading))
        sleep(500)


        telemetry.addLine("Pod position set")
        telemetry.update()


        waitForStart()

        while (opModeIsActive()) {

            localizer.update()
            telemetry.addData("position",Localizer.pose.toString())
            telemetry.update()

        }
    }
}
