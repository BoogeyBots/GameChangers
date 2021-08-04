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
        motor.setVelocityPIDFCoefficients(1.0, 0.0, 0.0, 0.0)
        //motor.targetPosition = (0.05 * COUNTS_PER_REV).toInt()
    }

    override fun loop() {

        while (opModeIsActive()) {
            val oldTarget = motor.targetPosition
            val newTarget = oldTarget - gamepad1.left_trigger * 2.0 + gamepad1.right_trigger * 2.0

            motor.targetPosition = (when {
                newTarget <= 0.0 -> 0
                newTarget >= 1000.0 -> 1000
                else -> newTarget.toInt()
            })

            telemetry.addData("TARGET POS", motor.targetPosition)
            telemetry.update()
        }

        motor.power = gamepad1.left_stick_x.toDouble()
        telemetry.addData("Pozitie Motor", motor.currentPosition)
        telemetry.update()
    }

}