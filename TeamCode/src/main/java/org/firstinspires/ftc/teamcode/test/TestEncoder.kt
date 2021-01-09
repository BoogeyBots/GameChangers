package org.firstinspires.ftc.teamcode.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.Robot
import org.firstinspires.ftc.teamcode.bbopmode.BBLinearOpMode
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer
import org.firstinspires.ftc.teamcode.modules.TestModule
import org.firstinspires.ftc.teamcode.util.Encoder

@TeleOp
class TestEncoder : BBLinearOpMode(){

    private var leftEncoder: Encoder ?= null
    private var rightEncoder: Encoder ?= null

    override val modules: Robot = Robot(setOf(TestModule(this)))
    override fun runOpMode() {
        leftEncoder = Encoder(hardwareMap.get(DcMotor::class.java, "leftRear") as DcMotorEx?)
        rightEncoder = Encoder(hardwareMap.get(DcMotor::class.java, "right  Front") as DcMotorEx?)

        while(!isStopRequested){

            telemetry.addData("Pozitie 1", encoderTicksToInches(leftEncoder!!.currentPosition))
            telemetry.addData("Pozitie 2", encoderTicksToInches(rightEncoder!!.currentPosition))
            telemetry.update()
        }
    }

    fun encoderTicksToInches(ticks: Int): Double {
        return StandardTrackingWheelLocalizer.WHEEL_RADIUS * 2 * Math.PI * StandardTrackingWheelLocalizer.GEAR_RATIO * ticks / StandardTrackingWheelLocalizer.TICKS_PER_REV
    }
}