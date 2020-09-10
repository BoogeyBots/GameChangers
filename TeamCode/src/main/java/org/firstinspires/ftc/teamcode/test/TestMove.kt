
package org.firstinspires.ftc.teamcode.test

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.Mecanum

@Autonomous(name = "TEST: Move")
class TestMove : LinearOpMode() {
    val robot = Mecanum(null)

    override fun runOpMode() {
        val trajectory = robot.trajectoryBuilder(Pose2d())
                .forward(60.0)
                .build()

        waitForStart()

        if(isStopRequested) return

        robot.followTrajectory(trajectory)

    }
}
