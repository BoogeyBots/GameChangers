package org.firstinspires.ftc.teamcode.robot.autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.Mecanum
import org.firstinspires.ftc.teamcode.Robot
import org.firstinspires.ftc.teamcode.bbopmode.BBLinearOpMode
import org.firstinspires.ftc.teamcode.bbopmode.get
import org.firstinspires.ftc.teamcode.modules.*
import org.firstinspires.ftc.teamcode.vision.TensorFlowObjectDetection
import org.opencv.core.Mat

@Autonomous(name = "testVictor")
class testVictor : BBLinearOpMode() {

    override val modules: Robot = Robot(setOf(WobbleGoalLift(this), ServoThrowerModule(this), MotorThrowerModule(this), Recognition(this), IntakeModule(this), ServoWobble(this)))

    override fun runOpMode() {
        val robot = Mecanum(hardwareMap)
        modules.modules.forEach() {
            it.init()
        }

        val nrRings = get<Recognition>().recognizeRings()

        val startPose = Pose2d(-63.0, -43.7)
        robot.poseEstimate = startPose

        waitForStart()

        val trajectory1 = robot.trajectoryBuilder(startPose)
                .splineTo(Vector2d(-25.50, -22.0), Math.toRadians(0.0))
                .splineTo(Vector2d(-1.5, -28.29), Math.toRadians(0.0))
                .build()
        robot.followTrajectory(trajectory1)
        val trajectory2 = robot.trajectoryBuilder(Pose2d(1.5, -28.290,.0))
                .lineTo(Vector2d(-1.5, -35.29))
                .build()

        robot.followTrajectory(trajectory2)
        val trajectory3 = robot.trajectoryBuilder(Pose2d(-1.5, -35.29, Math.toRadians(180.0)))
                .lineTo(Vector2d(-19.5, -37.29))
                .build()
        robot.turn(Math.toRadians(182.0))
        get<MotorThrowerModule>().setPower(0.65)
        wait(0.75)

        for(i in 1..3){
            get<ServoThrowerModule>().open()
            wait(0.2 )
            get<ServoThrowerModule>().close()
            wait(0.3)
        }
        get<IntakeModule>().move(true)
        robot.followTrajectory(trajectory3)
        wait(2.0)
        get<IntakeModule>().stop()
        val trajectory4 = robot.trajectoryBuilder(Pose2d(-19.5, -37.29, Math.toRadians(182.0)))
                .lineTo(Vector2d(-1.5, -37.29))
                .build()
        robot.followTrajectory(trajectory4)
        get<MotorThrowerModule>().setPower(0.65)
        wait(1.5)
        get<ServoThrowerModule>().open()
        wait(0.75)
        get<ServoThrowerModule>().close()
        val trajectory5 = robot.trajectoryBuilder(Pose2d(-1.5, -37.29, Math.toRadians(182.0)))
                .lineTo(Vector2d(7.00 ,-37.29))
                .build()
        robot.followTrajectory(trajectory5)

    }
}
