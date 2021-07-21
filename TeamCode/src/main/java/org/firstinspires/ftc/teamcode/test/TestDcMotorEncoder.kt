package org.firstinspires.ftc.teamcode.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.Robot
import org.firstinspires.ftc.teamcode.bbopmode.BBOpMode
import org.firstinspires.ftc.teamcode.modules.TestModule


@TeleOp()
class TestDcMotorEncoder : BBOpMode(){
    override val modules: Robot = Robot(setOf(TestModule(this)))
    lateinit var motor: DcMotorEx
    val isBusy get() = motor.isBusy
    val maxPos = 980
    val minPos = 0

    override fun init() {
        motor = hardwareMap.get(DcMotorEx::class.java, "brat")

        motor.targetPosition = 0
        motor.power = 0.0
        // ORIGINAL PIDF: p=9.999847 i=2.999954 d=0.000000 f=0.000000
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.mode = DcMotor.RunMode.RUN_TO_POSITION
        motor.setVelocityPIDFCoefficients(0.0, 0.0, 0.0, 0.0)
        //motor.targetPosition = (0.05 * COUNTS_PER_REV).toInt()
    }

    override fun loop() {
        motor.power = gamepad1.left_stick_x.toDouble()
        telemetry.addData("Pozitie Motor", motor.currentPosition)
        telemetry.update()
    }

}