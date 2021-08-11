package org.firstinspires.ftc.teamcode.modules

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime

class WobbleGoalModule(override val opMode: OpMode,val inAuto: Boolean) : RobotModule
{

    override var components: HashMap<String, HardwareDevice> = hashMapOf()
    val wobblegoal get() = get<Servo>("wobblegoal_servo")
    val wobblegoal2 get() = get<Servo>("wobblegoal_servo2")
    val wobblegoal_close get() = get<Servo>("wobblegoal_close")

    var wobblegoal_isUp: Boolean = true
    var wobblegoal_close_isClosed: Boolean = true
    val MOTOR_POWER = 0.25
    val time_elapsed = ElapsedTime()

    var x: Double = 0.7

    override fun init() {
        components["wobblegoal_servo"] = hardwareMap!!.get(Servo::class.java, "wobblegoal_servo")
        components["wobblegoal_servo2"] = hardwareMap!!.get(Servo::class.java, "wobblegoal_servo2")

        components["wobblegoal_close"] = hardwareMap!!.get(Servo::class.java, "wobblegoal_close")

        wobblegoal_close.position = 0.04
        if(inAuto) {
            while (x >= wobble1_up) {
                x -= 0.0005
                wobblegoal.position = x
            }
            while (x >= wobble2_up) {
                x -= 0.0005
                wobblegoal2.position = x
            }
        }
        else{
            wobblegoal.position = wobble1_init
            wobblegoal2.position = wobble2_init
        }
    }

    fun move_close(){
        if(!(inAuto)) {
            if (wobblegoal_close_isClosed && time_elapsed.milliseconds() > 500.0) {
                wobblegoal_close.position = 0.04
                wobblegoal_close_isClosed = false
                time_elapsed.reset()
            } else if (time_elapsed.milliseconds() > 500.0) {
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
            if (wobblegoal_isUp && time_elapsed.milliseconds() > 500.0) {
                wobblegoal.position = wobble1_down
                wobblegoal2.position = wobble2_down
                wobblegoal_isUp = false
                time_elapsed.reset()
            } else if (time_elapsed.milliseconds() > 500.0) {
                wobblegoal.position = wobble1_up
                wobblegoal2.position = wobble2_up
                wobblegoal_isUp = true
                time_elapsed.reset()
            }
        }
        else{
            if (wobblegoal_isUp ) {
                wobblegoal.position = wobble1_down
                wobblegoal2.position = wobble2_down
                wobblegoal_isUp = false
            } else {
                wobblegoal.position = wobble1_up
                wobblegoal2.position = wobble2_up
                wobblegoal_isUp = true
            }
        }
    }

    fun move_down(){
        wobblegoal_isUp = false
        wobblegoal.position = wobble1_init
        wobblegoal2.position = wobble2_init
    }

    fun move_endgame() {
        if (time_elapsed.milliseconds() > 500.0) {
            wobblegoal.position = wobble1_endgame
            wobblegoal2.position = wobble2_endgame
            wobblegoal_isUp = true
        }
    }

    fun move_auto(){
        wobblegoal_isUp = false
        wobblegoal.position = 0.34
        wobblegoal2.position = 0.646
    }

    companion object{
        val wobble1_init = 0.912 // 0.00 
        val wobble1_up = 0.6144 // 0.40
        val wobble1_down = 0.277 // 0.89
        val wobble1_endgame = 0.514

        val wobble2_init = 0.0794
        val wobble2_up = 0.37
        val wobble2_down = 0.712
        val wobble2_endgame = 0.46
    }
}
