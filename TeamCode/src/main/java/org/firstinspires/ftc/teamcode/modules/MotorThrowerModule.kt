    package org.firstinspires.ftc.teamcode.modules

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.PIDFCoefficients

    class MotorThrowerModule(override val opMode: OpMode) : RobotModule {
    override var components: HashMap<String, HardwareDevice> = hashMapOf()
    val motor get() = get<DcMotorEx>("thrower")

    override fun init() {
        components["thrower"] = hardwareMap!!.get(DcMotorEx::class.java, "thrower")
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDFCoefficients(70.0, 0.0 , 0.0 , 13.6))
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