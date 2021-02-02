package org.firstinspires.ftc.teamcode.test

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Robot
import org.firstinspires.ftc.teamcode.bbopmode.BBLinearOpMode
import org.firstinspires.ftc.teamcode.bbopmode.get
import org.firstinspires.ftc.teamcode.modules.ServoWobble
import org.firstinspires.ftc.teamcode.modules.WobbleGoalModule

@TeleOp
class TestWobble : BBLinearOpMode(){
    override val modules = Robot(setOf(WobbleGoalModule(this, inAuto = false), ServoWobble(this)))
    override fun runOpMode() {
        if(gamepad1.a){
            get<ServoWobble>().grab()
        }
        else if(gamepad1.b){
            get<ServoWobble>().ungrab()
        }
    }
}