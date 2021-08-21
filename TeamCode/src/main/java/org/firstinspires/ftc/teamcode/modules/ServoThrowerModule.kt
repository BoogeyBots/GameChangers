package org.firstinspires.ftc.teamcode.modules

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.Servo

class ServoThrowerModule(override val opMode: OpMode) : RobotModule {
    override var components: HashMap<String, HardwareDevice> = hashMapOf()
    val servo get() = get<Servo>("servo_thrower")

    override fun init() {
        components["servo_thrower"] = hardwareMap!!.get(Servo::class.java, "servo_thrower")
        servo.position = 0.7
    }

    fun close(){
        servo.position = 0.7
    }

    fun open(){
        servo.position = 0.37
    }
}