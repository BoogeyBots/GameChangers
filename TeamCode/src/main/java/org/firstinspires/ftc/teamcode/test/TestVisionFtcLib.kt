package org.firstinspires.ftc.teamcode.test

import com.arcrobotics.ftclib.vision.UGContourRingDetector
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvInternalCamera

@Disabled
@TeleOp
class TestVisionFtcLib() : LinearOpMode() {
    override fun runOpMode() {
        val detector = UGContourRingDetector(hardwareMap, OpenCvInternalCamera.CameraDirection.BACK, telemetry, true)

        UGContourRingDetector.PipelineConfiguration.CAMERA_WIDTH = 320

        UGContourRingDetector.PipelineConfiguration.CAMERA_HEIGHT = 240

        UGContourRingDetector.PipelineConfiguration.HORIZON = 100

        UGContourRingDetector.PipelineConfiguration.CAMERA_ORIENTATION = OpenCvCameraRotation.SIDEWAYS_RIGHT

        detector.init()

        waitForStart()

        while(opModeIsActive()){
            telemetry.addData("ceva", detector.height.toString())
            telemetry.update()
        }
    }

}