package org.firstinspires.ftc.teamcode.modules

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.util.ElapsedTime
import com.vuforia.CameraDevice
import org.firstinspires.ftc.robotcore.external.ClassFactory
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer
import org.firstinspires.ftc.robotcore.external.tfod.Recognition
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector
import org.firstinspires.ftc.teamcode.vision.TensorFlowObjectDetection


class Recognition(override val opMode: OpMode) : RobotModule {
    override var components: HashMap<String, HardwareDevice> = hashMapOf()

    override fun init() {
        initVuforia()
        initTfod()
    }

    enum class NrRings {
        ZERO,
        ONE,
        FOUR
    }

    fun initVuforia() {
        val parameters = VuforiaLocalizer.Parameters()

        parameters.vuforiaLicenseKey = TensorFlowObjectDetection.VUFORIA_KEY
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK

        vuforia = ClassFactory.getInstance().createVuforia(parameters)
    }

    fun initTfod() {

        val tfodMonitorViewId: Int = hardwareMap!!.appContext.resources.getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap!!.appContext.packageName)
        val tfodParameters = TFObjectDetector.Parameters(tfodMonitorViewId)
        tfodParameters.minResultConfidence = 0.71f //0.83


        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia)
        tfod!!.loadModelFromAsset(TensorFlowObjectDetection.TFOD_MODEL_ASSET, TensorFlowObjectDetection.LABEL_FIRST_ELEMENT, TensorFlowObjectDetection.LABEL_SECOND_ELEMENT)
        //tfod.setClippingMargins(200, 0, 250, 950);
        //tfod.setClippingMargins(200, 0, 250, 950);
        tfod!!.setZoom(1.0, 1.777)
    }

    fun recognizeRings(): NrRings {

        tfod?.activate()
        CameraDevice.getInstance().setFlashTorchMode(true)
        CameraDevice.getInstance().setFlashTorchMode(false)

        var foundRings = false
        var ringFound = ""

        while (!opModeIsActive) {
            if (tfod != null) {

                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                val updatedRecognitions: List<Recognition> = tfod!!.recognitions
                if (updatedRecognitions != null) {
                    telemetry!!.addData("# Object Detected", updatedRecognitions.size)
                    if (updatedRecognitions.isNotEmpty()) {
                        ringFound = updatedRecognitions[0].label
                        foundRings = true
                    }
                }
            }
        }


            tfod?.shutdown()

            return when (ringFound) {
                "Quad" -> NrRings.FOUR
                "Single" -> NrRings.ONE
                else -> NrRings.ZERO
            }

        }


        companion object {
            private var vuforia: VuforiaLocalizer? = null
            private var tfod: TFObjectDetector? = null
        }

        val opModeIsActive get() = linearOpMode.opModeIsActive()
    }
