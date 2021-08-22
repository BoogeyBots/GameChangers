package org.firstinspires.ftc.teamcode.test

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Robot
import org.firstinspires.ftc.teamcode.bbopmode.BBOpMode
import org.firstinspires.ftc.teamcode.bbopmode.get
import org.firstinspires.ftc.teamcode.modules.ServoThrowerModule
import org.firstinspires.ftc.teamcode.modules.TestModule

@Config
@TeleOp(group="drive")
class TestDcMotor : BBOpMode(){
    override val modules: Robot = Robot(setOf(TestModule(this), ServoThrowerModule(this)))
    lateinit var motor: DcMotorEx
    var isRunning = false
    var motorPower = 0.0
    var timex = ElapsedTime()

    override fun init() {
        motor = hardwareMap!!.get(DcMotorEx::class.java, "thrower")
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDFCoefficients(70.0, 0.0, 0.0, 13.6))
        get<ServoThrowerModule>().init()
    }

    override fun loop() {
        if(gamepad1.y and (motorPower < 1.0 && timex.seconds() > 0.1)) {
            motorPower += 0.05
            timex.reset()
        }
        else if(gamepad1.a and (motorPower > -1.0 && timex.seconds() > 0.1)){
            motorPower -= 0.05
            timex.reset()
        }
        if (gamepad1.x){
            motor.power = motorPower.toDouble()
        }
        else {
            motor.power = -gamepad1.left_stick_y.toDouble()
        }
        if(gamepad1.left_bumper){
            get<ServoThrowerModule>().close()
        }
        if (gamepad1.right_bumper){
            get<ServoThrowerModule>().open()
        }
        telemetry.addData("Motor theoretical twoMotorsPower: ", motorPower)
        telemetry.addData("Motor twoMotorsPower: ", motor.power)
        telemetry.addData("Encoder: ", motor.currentPosition)
        telemetry.update()
    }
}