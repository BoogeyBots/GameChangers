package org.firstinspires.ftc.teamcode.modules

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareDevice


class IntakeModule(override val opMode: OpMode) : RobotModule{
    override var components: HashMap<String, HardwareDevice> = hashMapOf()

    val motor1 get() = get<DcMotor>("intake")
    val motor2 get() = get<DcMotor>("intake2")

    override fun init() {
        components["intake"] = hardwareMap!!.get(DcMotor::class.java, "intake")
        components["intake2"] = hardwareMap!!.get(DcMotor::class.java, "intake2")
    }

    fun move(dir: Boolean){
        if(dir){
            motor1.power = -0.7
            motor2.power = -0.7
        }
        else{
            motor1.power = 0.7
            motor2.power = 0.7
        }
    }

    fun stop(){
        motor1.power = 0.0
        motor2.power = 0.0
    }


}