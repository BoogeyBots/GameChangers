package org.firstinspires.ftc.teamcode.vision;

import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@TeleOp
@Disabled
public class UGContourRingPipelineExample extends LinearOpMode {
    private static final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 240; // height of wanted camera resolution

    private static final int HORIZON = 100; // horizon value to tune

    private static final boolean DEBUG = false; // if debug is wanted, change to true

    private static final boolean USING_WEBCAM = false; // change to true if using webcam
    private static final String WEBCAM_NAME = ""; // insert webcam name from configuration if using webcam

    private com.arcrobotics.ftclib.vision.UGContourRingPipeline pipeline;
    private OpenCvCamera camera;

    private final int cameraMonitorViewId = this
            .hardwareMap
            .appContext
            .getResources().getIdentifier(
                    "cameraMonitorViewId",
                    "id",
                    hardwareMap.appContext.getPackageName()
            );

    @Override
    public void runOpMode() throws InterruptedException {
        if (USING_WEBCAM) {
            camera = OpenCvCameraFactory
                    .getInstance()
                    .createWebcam(hardwareMap.get(WebcamName.class, WEBCAM_NAME), cameraMonitorViewId);
        } else {
            camera = OpenCvCameraFactory
                    .getInstance()
                    .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        }

        camera.setPipeline(pipeline = new com.arcrobotics.ftclib.vision.UGContourRingPipeline(telemetry, DEBUG));

        com.arcrobotics.ftclib.vision.UGContourRingPipeline.Config.setCAMERA_WIDTH(CAMERA_WIDTH);

        com.arcrobotics.ftclib.vision.UGContourRingPipeline.Config.setHORIZON(HORIZON);

        camera.openCameraDeviceAsync(() -> camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT));

        waitForStart();

        while (opModeIsActive()) {
            String height = "[HEIGHT]" + " " + pipeline.getHeight();
            telemetry.addData("[Ring Stack] >>", height);
            telemetry.update();
        }
    }
}