package org.firstinspires.ftc.teamcode.Competition_Code.Subsystems

import android.util.Size
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

import org.firstinspires.ftc.teamcode.Competition_Code.Interfaces.Camera
import org.firstinspires.ftc.teamcode.Competition_Code.PinpointLocalizer.Localizer

import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.VisionProcessor
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import kotlin.math.cos
import kotlin.math.sin

class AprilTag(hardwareMap: HardwareMap) : Camera {

    enum class State {
        On, Off
    }

    var state = State.Off

    override val camera: WebcamName = hardwareMap.get(WebcamName::class.java, "Arducam")
    override val visionProcessor: VisionProcessor = AprilTagProcessor.Builder()
        .setDrawAxes(true)
        .setDrawTagOutline(true)
        .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
        .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
        .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
        .setLensIntrinsics(919.688, 919.688, 647.849 ,350.162)
        .build()


    override var visionPortal: VisionPortal

    init {
        val builder = VisionPortal.Builder()
            .setCamera(camera)
            .setCameraResolution(Size(1280, 720))
            .enableLiveView(false)
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            .addProcessor(visionProcessor)

        visionPortal = builder.build()
        visionPortal.stopStreaming()
    }

    fun searchForTag(): Vector2d {
        visionPortal.resumeStreaming()
        visionPortal.resumeLiveView()
        val currentDetections = (visionProcessor as AprilTagProcessor).detections


        for (detection in currentDetections) {
            if (detection.id == 12 || detection.id == 16) {
                //state = State.TagDiscovered
                val data = Vector2d(detection.ftcPose.x, detection.ftcPose.y)
                return cameraVector(fieldDistanceToTag(data))
            }
        }
        return Vector2d(0.0,0.0)
    }

    private fun fieldDistanceToTag(translateData: Vector2d): Vector2d {
        //todo measure Camera Offset
        val relX = translateData.x + 0.0
        val relY = translateData.y + 1.0
        require(relY > 0)

        val h = -Localizer.pose.heading.toDouble()
        val x = relX * cos(h) - relY * sin(h)
        val y = relX * sin(h) + relY * cos(h)

        return Vector2d(x, y)
    }

    private fun cameraVector(fieldDistanceToTag: Vector2d): Vector2d {
        val tagPose = Pair(-72.0, -48.0)
        val xPose: Double = tagPose.first - fieldDistanceToTag.x
        val yPose: Double = tagPose.second - fieldDistanceToTag.y
        return Vector2d(xPose, yPose)
    }

    fun update() {
        when (state) {
            State.On -> {
                getMotif()
            }

            State.Off -> {
                visionPortal.stopStreaming()
            }

        }
    }
    fun getMotif(): Int {
        visionPortal.resumeStreaming()
        visionPortal.resumeLiveView()
        val currentDetections = (visionProcessor as AprilTagProcessor).detections

        var motif: Int = 0

        for (detection in currentDetections) {
            if (detection.id == 21) {
                motif = 1
                        //gpp
            }
            if (detection.id == 22) {
                motif = 2
                //pgp
            }
            if (detection.id == 23) {
                motif = 3
                //ppg
            }
        }
        return motif
    }
}
