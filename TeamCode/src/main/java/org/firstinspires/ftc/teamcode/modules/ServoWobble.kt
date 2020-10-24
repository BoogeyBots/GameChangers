package org.firstinspires.ftc.teamcode.modules


import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.Servo

class ServoWobble(override val opMode: OpMode) : RobotModule {
    override var components: HashMap<String, HardwareDevice> = hashMapOf()
    private val servolinear get() = get<Servo>("servowobble")

    override fun init() {
        components["servowobble"] = hardwareMap!!.get(Servo::class.java, "servowobble")
        servolinear.position = UNGRAB_POS
    }

    fun grab() {
        servolinear.position = GRAB_POS
    }

    fun ungrab() {
        servolinear.position = UNGRAB_POS
    }

    companion object {
        const val GRAB_POS = 0.0030
        const val UNGRAB_POS = 1.00
        var resolution = 0.0030
    }
}