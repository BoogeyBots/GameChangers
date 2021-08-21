package org.firstinspires.ftc.teamcode.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Robot
import org.firstinspires.ftc.teamcode.bbopmode.BBOpMode
import org.firstinspires.ftc.teamcode.bbopmode.get
import org.firstinspires.ftc.teamcode.modules.TestModule
import org.firstinspires.ftc.teamcode.modules.WobbleGoalLift

@TeleOp
class TestBrat : BBOpMode() {
    override val modules: Robot = Robot(setOf(WobbleGoalLift(this)))

    override fun init() {
        get<WobbleGoalLift>().init()
    }

    override fun loop() {
        if(gamepad1.a){
            get<WobbleGoalLift>().goDown()
        }
        if(gamepad1.y){
            get<WobbleGoalLift>().goUp()
        }
    }
}