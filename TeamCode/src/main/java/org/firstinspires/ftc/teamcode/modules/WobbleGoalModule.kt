package org.firstinspires.ftc.teamcode.modules

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime

class WobbleGoalModule(override val opMode: OpMode,val inAuto: Boolean) : RobotModule
{

    override var components: HashMap<String, HardwareDevice> = hashMapOf()
    val wobblegoal get() = get<Servo>("wobblegoal_servo")
    val wobblegoal_close get() = get<Servo>("wobblegoal_close")

    var wobblegoal_isUp: Boolean = true
    var wobblegoal_close_isClosed: Boolean = true
    val MOTOR_POWER = 0.25
    val time_elapsed = ElapsedTime()

    var x: Double = 0.7

    override fun init() {
        components["wobblegoal_servo"] = hardwareMap!!.get(Servo::class.java, "wobblegoal_servo")
        components["wobblegoal_close"] = hardwareMap!!.get(Servo::class.java, "wobblegoal_close")
        wobblegoal_close.position = 0.04
        while(x>=0.4){
            x -= 0.0005
            wobblegoal.position = x
        }
    }

    fun move_close(){
        if(!(inAuto)) {
            if (wobblegoal_close_isClosed && time_elapsed.milliseconds() > 200.0) {
                wobblegoal_close.position = 0.04
                wobblegoal_close_isClosed = false
                time_elapsed.reset()
            } else if (time_elapsed.milliseconds() > 200.0) {
                wobblegoal_close.position = 0.7
                wobblegoal_close_isClosed = true
                time_elapsed.reset()
            }
        }
        else{
            if (wobblegoal_close_isClosed) {
                wobblegoal_close.position = 0.7
                wobblegoal_close_isClosed = false
            } else {
                wobblegoal_close.position = 0.04
                wobblegoal_close_isClosed = true
            }
        }
    }

    fun move_vertically(){
        if(!inAuto) {
            if (wobblegoal_isUp && time_elapsed.milliseconds() > 200.0) {
                wobblegoal.position = 0.06
                wobblegoal_isUp = false
                time_elapsed.reset()
            } else if (time_elapsed.milliseconds() > 200.0) {
                wobblegoal.position = 0.4
                wobblegoal_isUp = true
                time_elapsed.reset()
            }
        }
        else{
            if (wobblegoal_isUp ) {
                wobblegoal.position = 0.06
                wobblegoal_isUp = false
            } else {
                wobblegoal.position = 0.4
                wobblegoal_isUp = true
            }
        }
    }

    fun move_endgame() {
        if (time_elapsed.milliseconds() > 200.0) {
            wobblegoal.position = 0.25 //rares e sef
            wobblegoal_isUp = true
        }
    }
}
