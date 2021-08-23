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

@Autonomous(name = "testNationalaDreaptaROSU")
class testNationalaDreaptaROSU : BBLinearOpMode() {

    override val modules: Robot = Robot(setOf(WobbleGoalLift(this), ServoThrowerModule(this), MotorThrowerModule(this), Recognition(this), IntakeModule(this), ServoWobble(this)))

    override fun runOpMode() {
        val robot = Mecanum(hardwareMap)
        modules.modules.forEach() {
            it.init()
        }

        val nrRings = get<Recognition>().recognizeRings()

        val startPose = Pose2d(-63.0, -52.0)
        robot.poseEstimate = startPose

        waitForStart()

        if(nrRings == Recognition.NrRings.ZERO){
            val trajectory1 = robot.trajectoryBuilder(startPose)
                    .lineTo(Vector2d(-9.0, -52.0))
                    .build()
            robot.followTrajectory(trajectory1)
            robot.turn(Math.toRadians(-144.0))
            get<MotorThrowerModule>().setPower(0.587)
            wait(1.75)
            get<ServoThrowerModule>().open()
            wait(0.4)
            get<ServoThrowerModule>().close()
            robot.turn(Math.toRadians(-7.0))
            wait(0.5)
            get<ServoThrowerModule>().open()
            wait(0.4)
            get<ServoThrowerModule>().close()
            robot.turn(Math.toRadians(-3.5))
            wait(1.0)
            get<ServoThrowerModule>().open()
            wait(0.4)
            get<ServoThrowerModule>().close()
            get<MotorThrowerModule>().setPower(0.0)
            robot.turn(Math.toRadians(139.5))
            val trajectory2 = robot.trajectoryBuilder(Pose2d(-9.0, -53.0, Math.toRadians(0.0)))
                    .lineTo(Vector2d(3.0, -57.0))
                    .build()
            robot.followTrajectory(trajectory2)
            get<WobbleGoalLift>().goDown()
            wait(1.0)
            get<ServoWobble>().ungrab()
            wait(1.0)
            val trajectory3 = robot.trajectoryBuilder(Pose2d(3.0, -57.0, Math.toRadians(0.0)))
                    .lineTo(Vector2d(-21.0, -57.0))
                    .build()
            robot.followTrajectory(trajectory3)




        }
        else
            if(nrRings == Recognition.NrRings.FOUR){
                val trajectory1 = robot.trajectoryBuilder(startPose)
                        .lineTo(Vector2d(-9.0, -52.0))
                        .build()
                robot.followTrajectory(trajectory1)
                robot.turn(Math.toRadians(-144.0))
                get<MotorThrowerModule>().setPower(0.587)
                wait(1.75)
                get<ServoThrowerModule>().open()
                wait(0.4)
                get<ServoThrowerModule>().close()
                robot.turn(Math.toRadians(-7.0))
                wait(0.5)
                get<ServoThrowerModule>().open()
                wait(0.4)
                get<ServoThrowerModule>().close()
                robot.turn(Math.toRadians(-3.5))
                wait(1.0)
                get<ServoThrowerModule>().open()
                wait(0.4)
                get<ServoThrowerModule>().close()
                get<MotorThrowerModule>().setPower(0.0)
                robot.turn(Math.toRadians(139.5))
                val trajectory2 = robot.trajectoryBuilder(Pose2d(-9.0, -52.0, Math.toRadians(0.0)))
                        .lineTo(Vector2d(43.0, -55.0))
                        .build()
                robot.followTrajectory(trajectory2)
                get<WobbleGoalLift>().goDown()
                wait(1.0)
                get<ServoWobble>().ungrab()
                wait(1.0)
                get<WobbleGoalLift>().goBack()
                val trajectory3 = robot.trajectoryBuilder(Pose2d(43.0, -55.0, Math.toRadians(180.0)))
                        .lineTo(Vector2d(-18.0, -34.0))
                        .build()
                robot.turn(Math.toRadians(180.0))
                get<IntakeModule>().move(true)
                get<MotorThrowerModule>().setPower(0.64)
                robot.followTrajectory(trajectory3)
                val trajectory4 = robot.trajectoryBuilder(Pose2d(-18.0, -34.0, Math.toRadians(180.0)))
                        .lineTo(Vector2d(-30.0,-36.0 ))
                        .build()
                robot.followTrajectory(trajectory4)
                wait(2.0)
                get<IntakeModule>().stop()

                robot.turn(Math.toRadians(9.0))

                for(i in 1..3){
                    get<ServoThrowerModule>().open()
                    wait(0.2 )
                    get<ServoThrowerModule>().close()
                    wait(0.3)
                }
                val trajectory5 = robot.trajectoryBuilder(Pose2d(-18.0, -34.0, Math.toRadians(180.0)))
                        .lineTo(Vector2d(9.0,-55.0 ))
                        .build()
                robot.followTrajectory(trajectory5)




            }
    }
}
