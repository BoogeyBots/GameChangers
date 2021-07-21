package org.firstinspires.ftc.teamcode.robot.teleop

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Mecanum
import org.firstinspires.ftc.teamcode.Robot
import org.firstinspires.ftc.teamcode.bbopmode.get
import org.firstinspires.ftc.teamcode.modules.ServoWobble
import org.firstinspires.ftc.teamcode.modules.TestModule
import org.firstinspires.ftc.teamcode.modules.WobbleGoalModule

@TeleOp
@Disabled
class TeleOpFull : AutoTeleOp() {

    override val modules = Robot(setOf(TestModule(this)))
    lateinit var motor1: DcMotor
    lateinit var motor2: DcMotor
    var twoMotorsPower : Double = 0.0

    lateinit var servo: Servo
    val closed_servo = 0.0
    val opened_servo = 1.0
    var bool_closed = false

    lateinit var motor: DcMotorEx
    var isRunning = false

    var motorPower = 0.0
    var timex = ElapsedTime()


    override fun runOpMode() {
        val robot = Mecanum(hardwareMap)

        modules.modules.forEach {it.init()}

        motor = hardwareMap.get(DcMotorEx::class.java, "thrower")
        val batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next()
        motor1 = hardwareMap.get(DcMotor::class.java, "intake")
        motor2 = hardwareMap.get(DcMotor::class.java, "intake2")
        servo = hardwareMap.get(Servo::class.java, "servo_thrower")
            servo.position = 0.7

        while (!isStopRequested){
            driveRobot(robot, true)

            twoMotorsPower = gamepad1.right_trigger.toDouble()
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

}