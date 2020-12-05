package org.firstinspires.ftc.teamcode.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.Robot
import org.firstinspires.ftc.teamcode.bbopmode.BBOpMode
import org.firstinspires.ftc.teamcode.modules.TestModule

@TeleOp(name = "Test 2 DcMotors")
class Test2DcMotors : BBOpMode() {
    override val modules: Robot = Robot(setOf(TestModule(this)))
    lateinit var motor: DcMotor
    lateinit var motor2: DcMotor
    var power : Double = 0.0

    override fun init() {
        motor = hardwareMap.get(DcMotor::class.java, "leftRear")
        motor2 = hardwareMap.get(DcMotor::class.java, "leftFront")
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
    }

    override fun loop() {
        power = -gamepad1.right_stick_y.toDouble()
        motor.power = -power
        motor2.power = -power

    }
}
