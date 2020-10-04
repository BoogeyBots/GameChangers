package org.firstinspires.ftc.teamcode.modules

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.Servo

class WobbleGoalModule(override val opMode: OpMode) : RobotModule {
    override var components: HashMap<String, HardwareDevice> = hashMapOf()
    val wobblegoal get() = get<DcMotorEx>("wobblegoal")

    override fun init() {
        components["wobblegoal"] = hardwareMap!!.get(DcMotorEx::class.java, "wobblegoal")
    }

    fun move_goal(i: Float){
        wobblegoal.power = i.toDouble()
    }
}