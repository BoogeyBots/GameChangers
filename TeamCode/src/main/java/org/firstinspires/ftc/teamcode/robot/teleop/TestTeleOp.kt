package org.firstinspires.ftc.teamcode.robot.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Mecanum
import org.firstinspires.ftc.teamcode.Robot
import org.firstinspires.ftc.teamcode.modules.WobbleGoalModule
import java.util.*

@TeleOp
class TestTeleOp : BaseTeleOp() {

    override fun runOpMode() {
        val robot = Mecanum(hardwareMap)
        val modules : Robot = Robot(setOf(WobbleGoalModule(this)))

        modules.modules.forEach(){it.init()}


        while (!isStopRequested){
            driveRobot(robot)
            if(gamepad1.right_trigger > 0) {
                modules.get<WobbleGoalModule>().move_goal(gamepad1.right_trigger)
            }
            else if(gamepad1.left_trigger > 0){
                modules.get<WobbleGoalModule>().move_goal(gamepad1.left_trigger)
            }
        }

    }


}