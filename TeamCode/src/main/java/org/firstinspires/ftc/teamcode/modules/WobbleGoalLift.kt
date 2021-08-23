package org.firstinspires.ftc.teamcode.modules

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range

class WobbleGoalLift(override val opMode: OpMode) : RobotModule {
    override var components: HashMap<String, HardwareDevice> = hashMapOf()
    lateinit var motor: DcMotorEx
    val time_elapsed = ElapsedTime()

    var isUp = false


    override fun init() {
        motor = hardwareMap!!.get(DcMotorEx::class.java, "brat")
        motor.targetPosition = 0
        motor.power = 0.4
        // ORIGINAL PIDF: p=9.999847 i=2.999954 d=0.000000 f=0.000000
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.mode = DcMotor.RunMode.RUN_TO_POSITION
        motor.setVelocityPIDFCoefficients(15.0, 3.0, 0.0, 0.0)
        //motor.targetPosition = (0.05 * COUNTS_PER_REV).toInt()
    }

    fun goUp() {
        motor.targetPosition = 610
    }

    fun goDown() {
        motor.targetPosition = 1305
    }

    fun goMid() {
        motor.targetPosition = 1170
    }

    fun goEndGame(){
        motor.targetPosition = 765
    }
    fun goBack()
    {
        motor.targetPosition = 0
    }

    fun move() {
        if(isUp and (time_elapsed.milliseconds() > 500.0)) {
            goDown()
            isUp = false
            time_elapsed.reset()
        }
        if(!isUp and (time_elapsed.milliseconds() > 500.0)){
            goUp()
            isUp = true
            time_elapsed.reset()
        }
    }
}