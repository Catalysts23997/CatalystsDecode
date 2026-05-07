package org.firstinspires.ftc.teamcode.Competition_Code.Auto.Competition;

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import net.mccoder.ftvision.processors.ColorScannerFast
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.teamcode.Competition_Code.Actions.InterleagueActions
import org.firstinspires.ftc.teamcode.Competition_Code.AllianceColor
import org.firstinspires.ftc.teamcode.Competition_Code.Auto.AutoGlobals
import org.firstinspires.ftc.teamcode.Competition_Code.Auto.AutoPoints
import org.firstinspires.ftc.teamcode.Competition_Code.Auto.RunToExactForever
import org.firstinspires.ftc.teamcode.Competition_Code.PinpointLocalizer.Localizer
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Drivetrain
import org.firstinspires.ftc.teamcode.Competition_Code.Subsystems.Servo
import org.firstinspires.ftc.teamcode.Competition_Code.Utilities.Poses
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import java.util.concurrent.TimeUnit

@Autonomous(name = "BlueFar12", group = "Auto")
class BlueFar12 : LinearOpMode() {

    override fun runOpMode() {
        AutoGlobals.targetRobotPositon = AutoPoints.StartFarBlue.pose

        val localizer = Localizer(hardwareMap, AutoGlobals.targetRobotPositon)
        val drive = Drivetrain(hardwareMap, AllianceColor.Blue)
        val robot = InterleagueActions(hardwareMap, telemetry)


        robot.holder.state = Servo.State.STOP1
        robot.update()
        robot.launcher.baseRPM = 3300.0

        // start camera
        val cameraMonitorViewId = hardwareMap.appContext
            .getResources()
            .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName())

        val camera = OpenCvCameraFactory.getInstance()
            .createWebcam(
                hardwareMap.get<WebcamName?>(WebcamName::class.java, "Arducam"), cameraMonitorViewId
            )


        val detector = ColorScannerFast()
        camera.setPipeline(detector)

        camera.openCameraDeviceAsync(object : AsyncCameraOpenListener {
            override fun onOpened() {
                val exposureControl = camera.exposureControl
                exposureControl.mode = ExposureControl.Mode.Manual
                exposureControl.setExposure(30000, TimeUnit.MICROSECONDS)

                val gainControl = camera.gainControl
                gainControl.gain = 30

                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT)
                FtcDashboard.getInstance().startCameraStream(camera, 0.0)
            }

            override fun onError(errorCode: Int) {
            }
        })

        waitForStart()

        AutoGlobals.FarAuto = true
        AutoGlobals.AutonomousRan = true

        runBlocking(
            ParallelAction(
                object : Action {
                    override fun run(p: TelemetryPacket): Boolean {
                        if (isStopRequested) {
                            stop()
                        }

                        localizer.update()
                        RunToExactForever(AutoGlobals.targetRobotPositon)
                        AutoGlobals.locationOfRobot = Poses(Localizer.pose.x, Localizer.pose.y, Localizer.pose.heading)
                        //telemetry.addData("Can predict", detector.canPredictVelocity())
                        telemetry.addData("goalPos", AutoGlobals.targetRobotPositon)
                        telemetry.addData("heading", Localizer.pose.heading)
                        telemetry.addData("x", Localizer.pose.x)
                        telemetry.addData("y", Localizer.pose.y)
                        telemetry.update()
                        robot.update()
                        return true // keep looping
                    }
                },
                SequentialAction(
                    /*robot.StartShooter,
                    robot.StartIntake,

                    AutoPoints.LaunchFarBlue.runToExact(),

                    robot.ShootFar(),

                    AutoPoints.PreIntakeGPPFar.runToExact(),
                    robot.StartIntake,
                    AutoPoints.GPPIntakeFar.runToExact(),

                    AutoPoints.LaunchFarBlue.runToExact(),
                    robot.ShootFar(),*/

                    // BEGIN BALL DETECTOR

                    /*robot.StartIntake,
                    AutoPoints.PGPPreFar.runToExact(),
                    AutoPoints.PGPFar.runToExact(),
                    robot.WaitAction(125.0),

                    AutoPoints.LaunchFarBlue.runToExact(),
                    robot.ShootFar(),*/

                    robot.StopShooter,
                    AutoPoints.MoveFarBlue.runToExact(),

                    // ball detector
                    robot.BallDetectorTesting(detector)

                    /*robot.ActivateBallDetection(detector),
                    robot.WaitForBallDetection(detector),
                    robot.PredictBallLocationFromVelocity(detector),*/

                    // drive to ball
                    //robot.DriveToDetection(detector)

                    /*robot.StartIntake,
                    AutoPoints.PGPPreFar.runToExact(),
                    AutoPoints.PGPFar.runToExact(),
                    robot.WaitAction(125.0),

                    AutoPoints.LaunchFarBlue.runToExact(),
                    robot.ShootFar(),

                    AutoPoints.MoveFarBlue.runToExact()*/
                )
            )
        )

        /*while (opModeIsActive()) {
            this.telemetry.clear()
            this.telemetry.addLine("Running Ball Detector...")
            this.telemetry.update()

            if (detector.firstBall != null) {
                if (detector.firstBall.x < 100) {
                    this.telemetry.addLine("FOund!")
                    this.telemetry.update()
                }
            }
        }*/
    }
}
