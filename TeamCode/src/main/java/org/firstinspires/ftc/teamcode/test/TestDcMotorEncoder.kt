package org.firstinspires.ftc.teamcode.test

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.Robot
import org.firstinspires.ftc.teamcode.bbopmode.BBOpMode
import org.firstinspires.ftc.teamcode.modules.TestModule

@TeleOp()
@Disabled
class TestDcMotorEncoder : BBOpMode(){
    override val modules: Robot = Robot(setOf(TestModule(this)))
    lateinit var motor: DcMotorEx
    val isBusy get() = motor.isBusy
    val maxPos = 2500
    val minPos = -1

    override fun init() {
        motor = hardwareMap.get(DcMotorEx::class.java, "brat")

        //motor.targetPosition = motor.currentPosition
        //motor.power = 0.0
        // ORIGINAL PIDF: p=9.999847 i=2.999954 d=0.000000 f=0.000000
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        motor.targetPosition = 0

        motor.mode = DcMotor.RunMode.RUN_TO_POSITION
        motor.setVelocityPIDFCoefficients(15.0, 3.0, 2.0, 0.0)
        motor.power = 0.04
        //motor.targetPosition = (0.05 * COUNTS_PER_REV).toInt()
    }

    override fun loop() {
        if(gamepad1.a){
            goUp()
        }
        if (gamepad1.b){
            goDown()
        }

        telemetry.addData("TARGET POS", motor.targetPosition)
        telemetry.addData("CURRENT POS", motor.currentPosition)
        telemetry.update()
    }

    fun goUp() {
        if (motor.targetPosition in minPos  until (maxPos + 1))
            motor.targetPosition = Range.clip(motor.targetPosition + 15, minPos, maxPos)
                        motor.power = 0.3
    }
    fun goDown() {

        if (motor.targetPosition in minPos until (maxPos + 1))
            motor.targetPosition = Range.clip(motor.targetPosition - 15, minPos, maxPos)
        motor.power = 0.3

    }
}