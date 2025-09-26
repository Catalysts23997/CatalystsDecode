package org.firstinspires.ftc.teamcode.Competition_Code.Interfaces

interface SubSystems {
    fun update(){}
    val state: Any
        get() ={}
    fun update(gamepadInput: ArrayList<Float>){}
}