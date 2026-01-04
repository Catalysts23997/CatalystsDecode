package org.firstinspires.ftc.teamcode.Competition_Code.Utilities

import kotlin.math.abs

fun normalize (values: DoubleArray, limit: Double = 1.0): DoubleArray {
    var max: Double = 0.0
    for (value in values) {
        max = kotlin.math.max(max, abs(value))
    }

    if (max <= limit) return values

    for (i in values.indices){
        values[i] = values[i] * limit/max
    }
    return values
}