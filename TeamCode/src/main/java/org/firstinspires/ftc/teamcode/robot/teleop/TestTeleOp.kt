package org.firstinspires.ftc.teamcode.robot.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Mecanum
import org.firstinspires.ftc.teamcode.Robot
import org.firstinspires.ftc.teamcode.bbopmode.get
import org.firstinspires.ftc.teamcode.modules.ServoWobble
import org.firstinspires.ftc.teamcode.modules.WobbleGoalModule

@TeleOp
class TestTeleOp : AutoTeleOp() {

    override val modules = Robot(setOf(WobbleGoalModule(this), ServoWobble(this)))


    override fun runOpMode() {
        val robot = Mecanum(hardwareMap)

        modules.modules.forEach {it.init()}

        while (!isStopRequested){
            driveRobot(robot)
            /*
            if(gamepad1.right_trigger > 0) {
                modules.get<WobbleGoalModule>().move_goal(gamepad1.right_trigger)
            }
            else if(gamepad1.left_trigger > 0){
                modules.get<WobbleGoalModule>().move_goal(-gamepad1.left_trigger)
            }
            else{
                modules.get<WobbleGoalModule>().move_goal(0.0f)

            }
            */
            if(gamepad1.right_trigger > 0.1){
                get<WobbleGoalModule>().move_goal(gamepad1.right_trigger)
            }
            else if(gamepad1.left_trigger > 0.14){
                get<WobbleGoalModule>().move_goal(-gamepad1.left_trigger)
            }
            else{
                get<WobbleGoalModule>().move_goal(0.0.toFloat())
            }
            if(gamepad1.right_bumper){
                get<ServoWobble>().grab()
            }
            else if(gamepad1.left_bumper){
                get<ServoWobble>().ungrab()
            }


        }

    }

}