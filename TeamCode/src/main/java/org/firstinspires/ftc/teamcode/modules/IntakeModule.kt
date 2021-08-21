package org.firstinspires.ftc.teamcode.modules

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareDevice


class IntakeModule(override val opMode: OpMode) : RobotModule{
    override var components: HashMap<String, HardwareDevice> = hashMapOf()

    val motor get() = get<DcMotor>("intake2")

    override fun init() {
        components["intake2"] = hardwareMap!!.get(DcMotor::class.java, "intake2")
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    fun move(dir: Boolean){
        if(dir){
            motor.power = -1.0
        }
        else{
            motor.power = 1.0

        }
    }

    fun stop(){
        motor.power = 0.0
    }


}