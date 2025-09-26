package org.firstinspires.ftc.teamcode.Competition_Code.Subsystems


import android.util.Log
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.teamcode.Competition_Code.Interfaces.SubSystems
import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.PIDController
import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.PIDParams
import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.smoothGamepadInput
import kotlin.math.cos
import kotlin.math.sin


//todo Make Java version
class DrivetrainNoOdo(hwMap: HardwareMap) : SubSystems {
    enum class States {
        Manual, Auto
    }

    companion object {
        lateinit var instance: DrivetrainNoOdo
    }

    val Xpid = PIDController(PIDParams(0.2, 0.0001, 0.018, 0.0))
    val Ypid = PIDController(PIDParams(0.2, 0.0001, 0.02, 0.0))
    val Rpid = PIDController(PIDParams(1.4, 0.0001, 0.08, 0.0))

    override var state = States.Manual

    //todo note it will differ on new dt - (use customTest)
    val leftFront: DcMotor = hwMap.get(DcMotor::class.java, "backRight") //good
    val rightBack: DcMotor = hwMap.get(DcMotor::class.java, "frontLeft") // good
    val leftBack: DcMotor = hwMap.get(DcMotor::class.java, "frontRight") // good
    val rightFront: DcMotor = hwMap.get(DcMotor::class.java, "backLeft")
    val imu: IMU = hwMap.get(IMU::class.java, "imu")

    val parameters = IMU.Parameters(
        RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
            RevHubOrientationOnRobot.UsbFacingDirection.UP
        )
    )


    override fun update(gamepadInput: ArrayList<Float>) {

        when (state) {
            States.Auto -> {
                //leave empty
            }

            States.Manual -> {
                driveManual(gamepadInput)
            }
        }
    }
//    fun setPID(p: DoubleArray, i: DoubleArray, d: DoubleArray) {
//        val controllers = listOf(Xpid, Ypid, Rpid)
//        controllers.forEachIndexed { index, controller ->
//            controller.params = PIDParams(p[index], i[index], d[index], 0.0)
//        }
//    }

    init {
        leftBack.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        leftFront.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightBack.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightFront.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        leftBack.direction = DcMotorSimple.Direction.REVERSE
        leftFront.direction = DcMotorSimple.Direction.REVERSE
        rightFront.direction = DcMotorSimple.Direction.FORWARD
        rightBack.direction = DcMotorSimple.Direction.FORWARD


        imu.initialize(parameters)

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

    private fun driveManual(gamepadInput: ArrayList<Float>) {
        val input = gamepadInput.map { smoothGamepadInput(it.toDouble()) }
        Log.d("f", input.toString())
        val (axial, lateral, turn) = input

        val h = imu.robotYawPitchRollAngles
        val rotX = -axial * cos(h.yaw) - lateral * sin(h.yaw)
        val rotY = -axial * sin(h.yaw) + lateral * cos(h.yaw)

        //todo add rotational pid

        leftFront.power = (rotY - rotX + turn)
        leftBack.power = (rotY + rotX + turn)
        rightFront.power = (rotY + rotX - turn)
        rightBack.power = (rotY - rotX - turn)
    }



}
