package org.firstinspires.ftc.teamcode.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.Robot
import org.firstinspires.ftc.teamcode.bbopmode.BBLinearOpMode
import org.firstinspires.ftc.teamcode.modules.TestModule
import org.firstinspires.ftc.teamcode.modules.get

@TeleOp
class Test2Servos : BBLinearOpMode() {
    override val modules = Robot(setOf(TestModule(this)))
    var resolution = 0.0005
    var resChangeSpeed = 0.000000001
    lateinit var servo1: Servo
    lateinit var servo2: Servo

    // NOTES
    // A - 0.40 for grabbing the stones
    // B - 0.65
    // 1.0 servo right back is strâns, 0.06 trage tava
    // 0.845 trage tava stanga spate, 0.0 sus


    override fun runOpMode() {
        servo1 = hardwareMap.get(Servo::class.java, "servo_test")
        servo2 = hardwareMap.get(Servo::class.java, "servo_test2")

        waitForStart()

        while (opModeIsActive()) {
            resolution += when {
                gamepad1.dpad_up -> resChangeSpeed
                gamepad1.dpad_down -> -resChangeSpeed
                else -> 0.0
            }

            if (gamepad1.y) {
                servo1.position += resolution
            }
            if(gamepad1.a) {
                servo1.position -= resolution
            }
            if(gamepad1.x){
                servo2.position += resolution
            }
            if(gamepad1.b){
                servo2.position -= resolution
            }


            telemetry.addData("res", resolution)
            telemetry.addData("servo1: ", servo1.position)
            telemetry.addData("servo2: ", servo2.position)

            telemetry.update()
        }
    }
}