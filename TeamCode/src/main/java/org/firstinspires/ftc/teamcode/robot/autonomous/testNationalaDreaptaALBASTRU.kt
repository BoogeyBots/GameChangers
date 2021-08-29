package org.firstinspires.ftc.teamcode.robot.autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.hardware.ams.AMSColorSensor
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.Mecanum
import org.firstinspires.ftc.teamcode.Robot
import org.firstinspires.ftc.teamcode.bbopmode.BBLinearOpMode
import org.firstinspires.ftc.teamcode.bbopmode.get
import org.firstinspires.ftc.teamcode.modules.*
import org.firstinspires.ftc.teamcode.vision.TensorFlowObjectDetection
import org.opencv.core.Mat

@Autonomous(name = "testNationalaDreaptaALBASTRU")

class testNationalaDreaptaALBASTRU: BBLinearOpMode() {

    override val modules: Robot = Robot(setOf(WobbleGoalLift(this), ServoThrowerModule(this), MotorThrowerModule(this), Recognition(this), IntakeModule(this), ServoWobble(this)))

    override fun runOpMode() {
        val robot = Mecanum(hardwareMap)
        modules.modules.forEach() {
            it.init()
        }

        val nrRings = get<Recognition>().recognizeRings()

        val startPose = Pose2d(-63.0, 20.3)
        robot.poseEstimate = startPose

        waitForStart()

        if(nrRings == Recognition.NrRings.ZERO){
            val trajectory1 = robot.trajectoryBuilder(startPose)
                    .lineTo(Vector2d(-34.0, 20.3))
                    .build()
            robot.followTrajectory(trajectory1)

            robot.turn(Math.toRadians(-168.0))
            get<MotorThrowerModule>().setPower(0.64)
            wait(2.0)

            for(i in 1..3){
                get<ServoThrowerModule>().open()
                wait(0.3 )
                get<ServoThrowerModule>().close()
                wait(0.4)
            }
            get<MotorThrowerModule>().setPower(0.0)

            val trajectory2 = robot.trajectoryBuilder(Pose2d(-34.0, 20.3, Math.toRadians(0.0)))
                    .lineTo(Vector2d(12.0, 7.0))
                    .build()
            robot.followTrajectory(trajectory2)
            robot.turn(Math.toRadians(90.0))
            val trajectory3 = robot.trajectoryBuilder(Pose2d(12.0, 7.0, Math.toRadians( 90.0)))
                    .lineTo(Vector2d(16.0, 40.0))
                    .build()
            robot.followTrajectory(trajectory3)
            get<WobbleGoalLift>().goDown()
            wait(1.5)
            get<ServoWobble>().ungrab()
            wait(1.0)
            get<WobbleGoalLift>().goBack()
            val trajectory4 = robot.trajectoryBuilder(Pose2d(12.0, 40.0, Math.toRadians( 90.0)))
                    .lineTo(Vector2d(16.0, 9.0))
                    .build()
            robot.followTrajectory(trajectory4)

        }
        else
            if(nrRings == Recognition.NrRings.FOUR){
                val trajectory1 = robot.trajectoryBuilder(startPose)
                        .lineTo(Vector2d(-34.0, 20.3))
                        .build()
                robot.followTrajectory(trajectory1)

                robot.turn(Math.toRadians(-168.0))
                get<MotorThrowerModule>().setPower(0.64)
                wait(2.0)

                for(i in 1..3){
                    get<ServoThrowerModule>().open()
                    wait(0.3 )
                    get<ServoThrowerModule>().close()
                    wait(0.4)
                }
                robot.turn(Math.toRadians(225.0))
                get<ServoThrowerModule>().close()
                get<IntakeModule>().move(true)
                wait(2.0)
                val trajectory2 = robot.trajectoryBuilder(Pose2d(-34.0, 20.3, Math.toRadians(57.0)))
                        .lineTo(Vector2d(-17.50, 36.29))
                        .build()
                robot.followTrajectory(trajectory2)
                get<MotorThrowerModule>().setPower(0.65)
                wait(2.0)
                val trajectory3 = robot.trajectoryBuilder(Pose2d(-17.5, 36.29, Math.toRadians(57.0)))
                        .lineTo(Vector2d(-16.0,45.0 ))
                        .build()
                robot.followTrajectory(trajectory3)
                wait(1.0)
                val trajectory4 = robot.trajectoryBuilder(Pose2d(-16.0, 45.0, Math.toRadians(57.0)))
                        .lineTo(Vector2d(-15.0,46.0 ))
                        .build()
                robot.followTrajectory(trajectory4)
                wait(2.0)
                get<IntakeModule>().stop()

                robot.turn(Math.toRadians(125.0))

                for(i in 1..3){
                    get<ServoThrowerModule>().open()
                    wait(0.2 )
                    get<ServoThrowerModule>().close()
                    wait(0.3)
                }
                val trajectory5 = robot.trajectoryBuilder(Pose2d(-15.0, 46.0, Math.toRadians(0.0)))
                        .lineTo(Vector2d(50.0,40.0 ))
                        .build()
                robot.followTrajectory(trajectory5)
                robot.turn(Math.toRadians(45.0))
                get<WobbleGoalLift>().goDown()
                wait(1.0)
                get<ServoWobble>().ungrab()
                wait(0.5)
                get<WobbleGoalLift>().goBack()
                wait(1.0)
                robot.turn(Math.toRadians(-45.0))

                val trajectory6 = robot.trajectoryBuilder(Pose2d(50.0, 40.0, Math.toRadians(0.0)))
                        .lineTo(Vector2d(0.0,24.0 ))
                        .build()
                robot.followTrajectory(trajectory6)

            }
            else
                if(nrRings == Recognition.NrRings.ONE){
                    get<MotorThrowerModule>().setPower(0.64)
                    val trajectory1 = robot.trajectoryBuilder(startPose)
                            .lineTo(Vector2d(-34.0, 20.3))
                            .build()
                    robot.followTrajectory(trajectory1)

                    robot.turn(Math.toRadians(-168.0))

                    for(i in 1..3){
                        get<ServoThrowerModule>().open()
                        wait(0.3 )
                        get<ServoThrowerModule>().close()
                        wait(0.4)
                    }
                    robot.turn(Math.toRadians(228.0))
                    get<ServoThrowerModule>().close()
                    get<IntakeModule>().move(true)
                    val trajectory2 = robot.trajectoryBuilder(Pose2d(-34.0, 20.3, Math.toRadians(60.0)))
                            .lineTo(Vector2d(-20.50, 37.29))
                            .build()
                    robot.followTrajectory(trajectory2)
                    robot.turn(Math.toRadians(120.0))
                    //robot.turn(Math.toRadians(144.5))
                    get<MotorThrowerModule>().setPower(0.65)
                    wait(1.25)
                    get<IntakeModule>().stop()
                    robot.turn(Math.toRadians(8.0))
                    wait(1.0)
                    get<ServoThrowerModule>().open()
                    wait(0.2)
                    get<ServoThrowerModule>().close()
                    robot.turn(Math.toRadians(180.0))
                    wait(0.5)
                    val trajectory3 = robot.trajectoryBuilder(Pose2d(-17.50, 37.29, Math.toRadians(0.0)))
                            .lineTo(Vector2d(24.0, 31.29))
                            .build()
                    robot.followTrajectory(trajectory3)
                    get<WobbleGoalLift>().goDown()
                    wait(1.0)
                    get<ServoWobble>().ungrab()
                    wait(1.0)
                    get<WobbleGoalLift>().goBack()
                    val trajectory4 = robot.trajectoryBuilder(Pose2d(24.0, 31.29, Math.toRadians(0.0)))
                            .lineTo(Vector2d(9.0, 37.29))
                            .build()
                    robot.followTrajectory(trajectory4)

                }
    }
}

//testNationalaStangaROSU