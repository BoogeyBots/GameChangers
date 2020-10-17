
package org.firstinspires.ftc.teamcode.test

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.Mecanum
import org.firstinspires.ftc.teamcode.Robot

@Autonomous(name = "TEST: Move")
class TestMove : LinearOpMode() {

    override fun runOpMode() {
        val robot = Mecanum(hardwareMap)

        val trajectory = robot.trajectoryBuilder(Pose2d())
                .splineTo(Vector2d(20.0,15.0), Math.toRadians(270.00))
                .build()

        waitForStart()

        if(isStopRequested) return

        robot.followTrajectory(trajectory)

    }
}
