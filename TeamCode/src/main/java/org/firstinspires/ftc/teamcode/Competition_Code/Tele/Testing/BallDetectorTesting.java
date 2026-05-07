/*
 *
 * Copyright 2026 MC_Coder (mccoderdev)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

package org.firstinspires.ftc.teamcode.Competition_Code.Tele.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import net.mccoder.ftvision.processors.ColorScanner;
import net.mccoder.ftvision.processors.MLScanner;
import net.mccoder.ftvision.processors.TestingScanner;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "BallDetectorTesting", group = "Linear OpMode")
public class BallDetectorTesting extends LinearOpMode {

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        OpenCvWebcam camera = OpenCvCameraFactory.getInstance()
                .createWebcam(
                        hardwareMap.get(WebcamName.class, "Arducam"), cameraMonitorViewId);

        //camera.setPipeline(new MLScanner(hardwareMap, telemetry));
        camera.setPipeline(new ColorScanner(275, 100));

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                ExposureControl exposureControl = camera.getExposureControl();
                exposureControl.setMode(ExposureControl.Mode.Manual);
                exposureControl.setExposure(30000, TimeUnit.MICROSECONDS);

                GainControl gainControl = camera.getGainControl();
                gainControl.setGain(30);

                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(camera, 0);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        waitForStart();

        while (opModeIsActive()) {
            sleep(50);
        }

        /*VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(BuiltinCameraDirection.BACK);
        builder.addProcessor(coreScanner);
        builder.setCameraResolution(new Size(544, 288));
        visionPortal = builder.build();

        waitForStart();

        while (opModeIsActive()) {
            sleep(50);
        }*/
    }
}
