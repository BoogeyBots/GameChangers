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

@Autonomous(name = "testHighGoal")
class testHighGoal : BBLinearOpMode() {

    override val modules: Robot = Robot(setOf(WobbleGoalLift(this), ServoThrowerModule(this), MotorThrowerModule(this), Recognition(this), IntakeModule(this), ServoWobble(this)))

    override fun runOpMode() {
        val robot = Mecanum(hardwareMap)
        modules.modules.forEach() {
            it.init()
        }


        val startPose = Pose2d(-63.0, -52.0)
        robot.poseEstimate = startPose

        waitForStart()

        val trajectory1 = robot.trajectoryBuilder(startPose)
                .lineTo(Vector2d(-9.0, -52.0))
                .build()
        robot.followTrajectory(trajectory1)
        robot.turn(Math.toRadians(-165.0))
        get<MotorThrowerModule>().setPower(0.625)
        wait(1.75)

        for(i in 1..3){
            get<ServoThrowerModule>().open()
            wait(0.2 )
            get<ServoThrowerModule>().close()
            wait(0.3)
        }

    }
}
