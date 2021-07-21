package org.firstinspires.ftc.teamcode.robot.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Mecanum
import org.firstinspires.ftc.teamcode.Robot
import org.firstinspires.ftc.teamcode.modules.ServoWobble
import org.firstinspires.ftc.teamcode.modules.WobbleGoalModule

@TeleOp
class TestTeleOp : AutoTeleOp() {

    override fun runOpMode() {
        val robot = Mecanum(hardwareMap)
        val modules = Robot(setOf(WobbleGoalModule(this), ServoWobble(this)))

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

             if(gamepad1.dpad_down){
                 modules.get<WobbleGoalModule>().move()
             }

             */
            if(gamepad1.left_trigger > 0.0){
                modules.get<WobbleGoalModule>().move_continuos(gamepad1.left_trigger)
            }
            else if(gamepad1.right_trigger > 0.0){
                modules.get<WobbleGoalModule>().move_continuos(-gamepad1.right_trigger)
            }

            if(gamepad1.right_bumper){
                modules.get<ServoWobble>().grab()
            }
            else if(gamepad1.left_bumper){
                modules.get<ServoWobble>().ungrab()
            }

            telemetry.addData("position: ", modules.get<WobbleGoalModule>().wobblegoal.currentPosition)
            telemetry.addData("PID ", modules.get<WobbleGoalModule>().wobblegoal.getPIDFCoefficients(modules.get<WobbleGoalModule>().wobblegoal.mode))

        }

    }

}