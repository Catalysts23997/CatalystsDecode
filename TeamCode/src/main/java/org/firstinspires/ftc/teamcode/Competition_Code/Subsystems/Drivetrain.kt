package org.firstinspires.ftc.teamcode.Competition_Code.Subsystems

import android.util.Log
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.Competition_Code.AllianceColor

import org.firstinspires.ftc.teamcode.Competition_Code.PinpointLocalizer.Localizer

import org.firstinspires.ftc.teamcode.Competition_Code.Interfaces.SubSystems
import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.PIDController
import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.PIDParams
import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.smoothGamepadInput
import kotlin.math.cos
import kotlin.math.sin


class Drivetrain(hwMap: HardwareMap, alliance: AllianceColor) : SubSystems {

    companion object {
        lateinit var instance: Drivetrain
    }

    val Xpid = PIDController(PIDParams(0.2, 0.0001, 0.018, 0.0))
    val Ypid = PIDController(PIDParams(0.2, 0.0001, 0.02, 0.0))
    val Rpid = PIDController(PIDParams(1.4, 0.0001, 0.08, 0.0))

    val leftFront: DcMotor = hwMap.get(DcMotor::class.java, "frontLeft") //good
    val rightBack: DcMotor = hwMap.get(DcMotor::class.java, "backRight") // good
    val leftBack: DcMotor = hwMap.get(DcMotor::class.java, "backLeft") // good
    val rightFront: DcMotor = hwMap.get(DcMotor::class.java, "frontRight")

    var offset: Double = 0.0

    var slowMode = false

    override fun update(gamepadInput: ArrayList<Float>) {

        driveManual(gamepadInput)

    }

    init {
        leftBack.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        leftFront.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightBack.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightFront.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        leftBack.direction = DcMotorSimple.Direction.REVERSE
        leftFront.direction = DcMotorSimple.Direction.REVERSE
        rightFront.direction = DcMotorSimple.Direction.FORWARD
        rightBack.direction = DcMotorSimple.Direction.FORWARD

        offset = if (alliance == AllianceColor.Red){
            Math.PI/2
        } else -Math.PI/2

        instance = this
    }

    fun WheelDebugger(x: Int) {
        val y = arrayListOf(leftBack, leftFront, rightBack, rightFront)
        y[x].power = .5
    }

    fun StopRobot() {
        leftFront.power = 0.0
        leftBack.power = 0.0
        rightFront.power = 0.0
        rightBack.power = 0.0
    }

    fun slowToggle() {
        slowMode = !slowMode
    }

    private fun driveManual(gamepadInput: ArrayList<Float>) {
        val input = gamepadInput.map { smoothGamepadInput(it.toDouble()) }
        Log.d("f", input.toString())
        val (axial, lateral, turn) = input

        val h = -Localizer.pose.heading + offset
        val rotX = -axial * cos(h) - lateral * sin(h)
        val rotY = -axial * sin(h) + lateral * cos(h)


        val k = if(slowMode) 0.5
        else 1.0

        leftFront.power = k*(rotY - rotX + turn)
        leftBack.power = k*(rotY + rotX + turn)
        rightFront.power = k*(rotY + rotX - turn)
        rightBack.power = k*(rotY - rotX - turn)
    }
}
