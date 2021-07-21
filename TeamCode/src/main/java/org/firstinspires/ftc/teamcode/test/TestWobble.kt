package org.firstinspires.ftc.teamcode.test

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Robot
import org.firstinspires.ftc.teamcode.bbopmode.BBLinearOpMode
import org.firstinspires.ftc.teamcode.modules.ServoWobble
import org.firstinspires.ftc.teamcode.modules.WobbleGoalModule

@TeleOp()
class TestWobble : BBLinearOpMode(){
    override val robot = Robot(setOf(WobbleGoalModule(this), ServoWobble(this)))


    override fun runOpMode() {
        if(gamepad1.left_trigger > 0.0){
            robot.get<WobbleGoalModule>().move_continuos(gamepad1.left_trigger)
        }
        else if(gamepad1.right_trigger > 0.0){
            robot.get<WobbleGoalModule>().move_continuos(gamepad1.right_trigger)
        }
        if(gamepad1.a){
            robot.get<ServoWobble>().grab()
        }
        else if(gamepad1.b){
            robot.get<ServoWobble>().ungrab()
        }
    }



}