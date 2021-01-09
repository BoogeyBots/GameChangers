package org.firstinspires.ftc.teamcode.modules

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.util.ElapsedTime

class WobbleGoalModule(override val opMode: OpMode) : RobotModule
{
    override var components: HashMap<String, HardwareDevice> = hashMapOf()
    val wobblegoal get() = get<DcMotorEx>("wobblegoal")
    var isUp: Boolean = true
    val MOTOR_POWER = 0.25
    val time_elapsed = ElapsedTime()

    override fun init() {
        components["wobblegoal"] = hardwareMap!!.get(DcMotorEx::class.java, "wobblegoal")
        wobblegoal.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        /*
        wobblegoal.targetPosition = 210
        wobblegoal.twoMotorsPower = MOTOR_POWER
        wobblegoal.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        wobblegoal.mode = DcMotor.RunMode.RUN_TO_POSITION
        wobblegoal.targetPositionTolerance = 5
        wobblegoal.setVelocityPIDFCoefficients(8.0, 0.85, 6.0, 0.0)
         */
        wobblegoal.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    fun move_goal(i: Float){
        wobblegoal.power = i.toDouble()
    }

    /*
    fun move() {
        if(time_elapsed.seconds() > 1.0) {
            wobblegoal.twoMotorsPower = MOTOR_POWER
            if (isUp) {
                wobblegoal.targetPosition = 451
                isUp = false
            } else {
                wobblegoal.targetPosition = 210
                isUp = true
            }
            time_elapsed.reset()
        }

     */
}
