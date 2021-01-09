package org.firstinspires.ftc.teamcode.robot.autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.Mecanum

@Autonomous(name = "TwoWobbleGoal")
class TwoWobbleGoal : LinearOpMode(){


    override fun runOpMode() {
        val robot = Mecanum(hardwareMap)

        val startPose = Pose2d(-60.0, 48.0)
        robot.poseEstimate = startPose

        val trajectory1 = robot.trajectoryBuilder(startPose)
                .splineTo(Vector2d(34.0, 36.0) ,0.0)
                .build()


        val trajectory2 = robot.trajectoryBuilder(trajectory1.end(), true)
                .splineTo(Vector2d(-60.0, 48.0 ), Math.toRadians(180.0))
                    .build()

        val trajectory3 = robot.trajectoryBuilder(trajectory2.end())
                .splineTo(Vector2d(-30.0, 49.0 ), Math.toRadians(0.0))
                .build()

        waitForStart()

        robot.followTrajectory(trajectory1)
        robot.followTrajectory(trajectory2)
        robot.followTrajectory(trajectory3)
    }
}