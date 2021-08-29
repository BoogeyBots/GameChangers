package org.firstinspires.ftc.teamcode.modules


import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime

class ServoWobble(override val opMode: OpMode) : RobotModule {
    override var components: HashMap<String, HardwareDevice> = hashMapOf()
    private val servolinear get() = get<Servo>("servowobble")
    val time_elapsed = ElapsedTime()
    var is_closed = false

    override fun init() {
        components["servowobble"] = hardwareMap!!.get(Servo::class.java, "servowobble")
        servolinear.position = 0.1
    }

    fun grab() {
        servolinear.position = GRAB_POS
    }

    fun ungrab() {
        servolinear.position = UNGRAB_POS
    }

    fun move() {
        if (is_closed and (time_elapsed.milliseconds() > 500.0)) {
            ungrab()
            time_elapsed.reset()
            is_closed = false
        }
        if (!is_closed and (time_elapsed.milliseconds() > 500.0)) {
            grab()
            time_elapsed.reset()
            is_closed = true
        }
    }

    companion object {
        const val GRAB_POS = 0.05
        const val UNGRAB_POS = 0.6
    }
}