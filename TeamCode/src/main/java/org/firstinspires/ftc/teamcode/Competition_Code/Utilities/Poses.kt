package org.firstinspires.ftc.teamcode.Competition_Code.Utilities

import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.Vector2d
import kotlin.math.abs

data class Poses(var x: Double, var y: Double, var heading: Double){
    constructor(vector2d: Vector2d, rotation: Double) : this(vector2d.x,vector2d.y, rotation)
    constructor(vector2d: Vector2d, rotation: Rotation2d) : this(vector2d.x,vector2d.y, rotation.toDouble())

    fun add(pose: Poses) {
        this.x += pose.x
        this.y += pose.y
        this.heading += pose.heading
        this.heading = Angles.wrap(this.heading) // wrap heading
    }

    override fun equals(other: Any?): Boolean {
        if(other !is Poses) return false // throw exception

        return (abs(other.x - this.x )<= 2.0 &&
                abs( other.y - this.y )<= 2.0 &&
                abs(Angles.wrap(other.heading - this.heading)) <= Math.toRadians(4.0))
    }

    override fun toString(): String {
        return "(${this.x}, ${this.y}, ${this.heading}) = (x,y,r)"
    }
}