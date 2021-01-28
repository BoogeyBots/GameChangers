package org.firstinspires.ftc.teamcode.modules

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareDevice

class MotorThrowerModule(override val opMode: OpMode) : RobotModule {
    override var components: HashMap<String, HardwareDevice> = hashMapOf()
    val motor get() = get<DcMotor>("thrower")

    override fun init() {
        components["thrower"] = hardwareMap!!.get(DcMotor::class.java, "thrower")
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        /*
        val motorConfigurationType = motor.motorType.clone()
        motorConfigurationType.achieveableMaxRPMFraction = 1.0
        motor.motorType = motorConfigurationType

        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        q*/
    }



    fun setPower(pw: Double){
        motor.power = pw
    }
}