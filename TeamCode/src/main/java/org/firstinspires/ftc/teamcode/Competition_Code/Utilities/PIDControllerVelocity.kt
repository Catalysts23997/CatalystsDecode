package org.firstinspires.ftc.teamcode.Competition_Code.Utilities

import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.PI
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sin

class PIDControllerVelocity(private var params: PIDParams) {
    private var prevError = 0.0
    private var integral = 0.0
    private val timer = ElapsedTime()
    var pastTime = 0.0

    @JvmOverloads // This fixes problems using this code from Java (not Kotlin)
    fun calculate(
        targetVelocity: Double,
        currentVelocity: Double
    ): Double {
        val dt = timer.seconds() - pastTime
        val error = targetVelocity - currentVelocity
        integral += (error * dt)

        val derivative = (error - prevError) / dt
        prevError = error
        pastTime = timer.seconds()

        val ff =
            if (params.kf != 0.0) params.kf * targetVelocity else 0.0


        val controlEffort =
            ((derivative * params.kd + integral * params.ki + error * params.kp) + ff).coerceIn(
                -1.0,
                1.0
            )

//        Log.d("errorsss", ff.toString())
//        Log.d("errorsss", Math.toDegrees(armAngle).toString())
        return controlEffort
    }

    fun calculate(
        targetVelocity: Double,
        currentVelocity: Double, voltage:Double
    ): Double {
        val dt = timer.seconds() - pastTime
        val error = targetVelocity - currentVelocity
        integral += (error * dt)

        val derivative = (error - prevError) / dt
        prevError = error
        pastTime = timer.seconds()

        val ff =
            if (params.kf != 0.0) params.kf * targetVelocity * 13.0/voltage else 0.0


        val controlEffort =
            ((derivative * params.kd + integral * params.ki + error * params.kp) + ff).coerceIn(
                -1.0,
                1.0
            )

//        Log.d("errorsss", ff.toString())
//        Log.d("errorsss", Math.toDegrees(armAngle).toString())
        return controlEffort
    }

    fun setPID(p: Double, i: Double, d: Double, f: Double) {
        params = PIDParams(p, i, d, f)
    }
    fun setPID(pidParams: PIDParams) {
        params = pidParams
    }
}
