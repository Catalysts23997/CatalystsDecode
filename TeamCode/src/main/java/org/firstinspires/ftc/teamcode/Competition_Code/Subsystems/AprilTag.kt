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

    /**
     * The different states that the subsystem can be in
     */
    enum class State {
        On, Off
    }

    /**
     * The current state of the April Tag subsystem
     */
    var state = State.Off

    /**
     * The instance of our camera
     */
    override val camera: WebcamName = hardwareMap.get(WebcamName::class.java, "Arducam")
    override val visionProcessor: VisionProcessor = AprilTagProcessor.Builder()
        // Set settings on the camera
        .setDrawAxes(true)
        .setDrawTagOutline(true)
        // Tell the camera what kind of tag we are looking for
        .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
        // Give the camera a list of locations that the April tags
        // will be at
        .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
        // We want the output to be in Inches and Radians
        .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
        // Set lens settings
        .setLensIntrinsics(919.688, 919.688, 647.849 ,350.162)
        // Create a new instance of our camera with all of the options
        // we set
        .build()

    /**
     * The Vision Portal, which controls the camera
     */
    override var visionPortal: VisionPortal

    // The code that will run when the instance is first created
    init {
        // Create a new Vision Portal
        val builder = VisionPortal.Builder()
            // Give it our camera
            .setCamera(camera)
            // Set the camera resolution
            .setCameraResolution(Size(1280, 720))
            .enableLiveView(false)
            // Tell the camera that we want to get images in
            // Motion JPEG format
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            // Add our vision processor
            .addProcessor(visionProcessor)

        visionPortal = builder.build()
        visionPortal.stopStreaming()
    }

    /**
     * Search for a tag
     */
    fun searchForTag(): Vector2d {
        // Tell our camera to start streaming video data
        // to our code!
        visionPortal.resumeStreaming()
        visionPortal.resumeLiveView()
        // Ask the camera if it sees any april tags
        val currentDetections = (visionProcessor as AprilTagProcessor).detections

        // Loop through every tag that it sees
        for (detection in currentDetections) {
            // Check if it the detected tag is one of the tags
            // that we need to find.
            if (detection.id == 12 || detection.id == 16) {
                //state = State.TagDiscovered
                // Create a new vector from the camera's view to return to the
                // caller
                val data = Vector2d(detection.ftcPose.x, detection.ftcPose.y)
                return cameraVector(fieldDistanceToTag(data))
            }
        }

        // We didn't find anything.
        return Vector2d(0.0,0.0)
    }

    /**
     * This function returns the distance the detected tag is from the
     * robot
     */
    private fun fieldDistanceToTag(translateData: Vector2d): Vector2d {
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

    fun getAngle(): Double {
        visionPortal.resumeStreaming()
        visionPortal.resumeLiveView()
        val currentDetections = (visionProcessor as AprilTagProcessor).detections
        var angle = 0.0
        for (detection in currentDetections) {
            angle = detection.ftcPose.yaw
        }
        return angle
    }
}
