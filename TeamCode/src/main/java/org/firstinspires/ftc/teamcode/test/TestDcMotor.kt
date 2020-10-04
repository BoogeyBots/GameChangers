package org.firstinspires.ftc.teamcode.test

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.Robot
import org.firstinspires.ftc.teamcode.bbopmode.BBOpMode
import org.firstinspires.ftc.teamcode.modules.TestModule

@Config
@TeleOp(group="drive")
public class TestDcMotor : BBOpMode(){
    override val robot: Robot = Robot(setOf(TestModule(this)))
    lateinit var motor: DcMotorEx

    override fun init() {
        motor = hardwareMap.get(DcMotorEx::class.java, "wobblegoal")
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    override fun loop() {
        motor.power = -gamepad1.left_stick_y.toDouble() 
    }
}