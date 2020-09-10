package org.firstinspires.ftc.teamcode.robot.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Mecanum
import java.util.*

@TeleOp
class TestTeleOp : BaseTeleOp() {

    override fun runOpMode() {
        val robot = Mecanum(null)

        while (!isStopRequested){
            driveRobot(robot)
        }

    }

    companion object{
    }

}