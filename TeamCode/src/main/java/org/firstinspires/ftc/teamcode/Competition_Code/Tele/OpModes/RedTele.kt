package org.firstinspires.ftc.teamcode.Competition_Code.Tele.OpModes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Competition_Code.AllianceColor

@TeleOp(name = "NewRedTele", group = "Linear OpMode")
class RedTele : LinearOpMode() {
    override fun runOpMode() {
        // Create the instance of our BaseTele
        val baseTele: BaseTele = BaseTele(this, AllianceColor.Red)

        // There is no setup code to run here because it is all ran
        // in BaseTele!

        // Wait for the user to press start
        waitForStart()

        // Start teleop
        baseTele.start()

        while (opModeIsActive()) {
            // Tick teleop
            baseTele.update()
        }
    }
}