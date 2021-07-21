package org.firstinspires.ftc.teamcode.modules

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range

class WobbleGoalLift(override val opMode: OpMode) : RobotModule {
    override var components: HashMap<String, HardwareDevice> = hashMapOf()
    val wobblegoal get() = get<DcMotorEx>("wobblegoal")
    var isUp: Boolean = true
    val MOTOR_POWER = 0.25
    val time_elapsed = ElapsedTime()
    val minPos = 0
    val maxPos = 0
    val changePos = 0

    override fun init() {
        components["wobblegoal"] = hardwareMap!!.get(DcMotorEx::class.java, "wobblegoal")
        wobblegoal.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        wobblegoal.targetPosition = 0
        wobblegoal.power = 0.4
        wobblegoal.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        wobblegoal.mode = DcMotor.RunMode.RUN_TO_POSITION
        wobblegoal.setVelocityPIDFCoefficients(5.0, 4.0, 2.0, 0.0)
    }

    fun goUp(){
        if (wobblegoal.targetPosition in (minPos) until (maxPos + 1))
            wobblegoal.targetPosition = Range.clip(wobblegoal.targetPosition + changePos, minPos, maxPos)
        wobblegoal.power = 0.4
    }
    fun goDown(){
        if (wobblegoal.targetPosition in (minPos) until (maxPos + 1))
            wobblegoal.targetPosition = Range.clip(wobblegoal.targetPosition - changePos, minPos, maxPos)
        wobblegoal.power = 0.4
    }
}