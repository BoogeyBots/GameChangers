
package org.firstinspires.ftc.teamcode.test

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.Mecanum
import org.firstinspires.ftc.teamcode.Robot

@Autonomous(name = "TEST: Move")
class TestMove : LinearOpMode() {
    val robot = Robot(this, setOf(Mecanum(this)))
    val power = 0.95
    override fun runOpMode() {
        robot.modules.forEach { it.init() }

        waitForStart()

        robot.get<Mecanum>().forward(-72.0, power, timeout = 5.0)
        sleep(500)
        robot.get<Mecanum>().rotate(-90.0, power, timeout = 5.0)
        sleep(500)
        robot.get<Mecanum>().sideways(-72.0, power, 5.0)
        sleep(500)


/*
		get<Mecanum>().sideways(24.0,1.0,2.0)
		get<Mecanum>().sideways(-24.0,1.0,2.0)
		get<Mecanum>().sideways(5.0,1.0,4.0)
		get<Mecanum>().sideways(-5.0,1.0,4.0)
*/

    }
}