package org.firstinspires.ftc.teamcode.test

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Robot
import org.firstinspires.ftc.teamcode.bbopmode.BBOpMode
import org.firstinspires.ftc.teamcode.modules.TestModule

@Config
@Disabled
@TeleOp(group="drive")
class Test4Motoare : BBOpMode(){
    lateinit var motor1: DcMotor
    lateinit var motor2: DcMotor
    var twoMotorsPower : Double = 0.0

    lateinit var servo: Servo
    val closed_servo = 0.0
    val opened_servo = 1.0
    var bool_closed = false

    override val modules: Robot = Robot(setOf(TestModule(this)))
    lateinit var motor: DcMotorEx
    var isRunning = false

    var motorPower = 0.0
    var timex = ElapsedTime()

    override fun init() {
        motor = hardwareMap.get(DcMotorEx::class.java, "thrower")
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        val motorConfigurationType = motor.motorType.clone()
        motorConfigurationType.achieveableMaxRPMFraction = 1.0
        motor.motorType = motorConfigurationType
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        val batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next()
        motor1 = hardwareMap.get(DcMotor::class.java, "intake")
        motor2 = hardwareMap.get(DcMotor::class.java, "intake")
        servo = hardwareMap.get(Servo::class.java, "servo_thrower")
        servo.position = 0.7
    }

    override fun loop() {
        twoMotorsPower = -gamepad1.right_trigger.toDouble()
        motor1.power = -twoMotorsPower
        motor2.power = -twoMotorsPower

        if(gamepad1.right_bumper ){
            servo.position = 0.4
        }
        if(gamepad1.left_bumper){
            servo.position = 0.7
        }

        if(gamepad1.y and (motorPower < 1 && timex.seconds() > 0.1)) {
            motorPower += 0.05
            timex.reset()
        }
        else if(gamepad1.a and (motorPower > 0 && timex.seconds() > 0.1)){
            motorPower -= 0.05
            timex.reset()
        }
        if (gamepad1.x){
            motor.power = motorPower.toDouble()
        }
        else {
            motor.power = -gamepad1.left_trigger.toDouble()
        }
        telemetry.addData("Motor theoretical twoMotorsPower: ", motorPower)
        telemetry.addData("Motor twoMotorsPower: ", motor.power)
        telemetry.update()
    }
}