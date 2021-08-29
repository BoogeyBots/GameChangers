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

@Autonomous(name = "testHighGoalAlbastru")
class testHighGoalAlbastru : BBLinearOpMode() {

    override val modules: Robot = Robot(setOf(WobbleGoalLift(this), ServoThrowerModule(this), MotorThrowerModule(this), Recognition(this), IntakeModule(this), ServoWobble(this)))

    override fun runOpMode() {
        val robot = Mecanum(hardwareMap)
        modules.modules.forEach() {
            it.init()
        }

        val nrRings = get<Recognition>().recognizeRings()

        val startPose = Pose2d(-63.0, 52.0)
        robot.poseEstimate = startPose

        waitForStart()

        if(nrRings == Recognition.NrRings.ZERO) {

            //wait(5.0)

            val trajectory1 = robot.trajectoryBuilder(startPose)
                    .lineTo(Vector2d(-9.0, 52.0))
                    .addTemporalMarker(1.5){
                        get<MotorThrowerModule>().setPower(0.64)
                    }
                    .build()
            robot.followTrajectory(trajectory1)
            robot.turn(Math.toRadians(165.0))
            wait(1.75)

            for (i in 1..3) {
                wait(0.3)
                get<ServoThrowerModule>().open()
                wait(0.2)
                get<ServoThrowerModule>().close()
            }

            val trajectory2 = robot.trajectoryBuilder(Pose2d(-9.0, 53.0, Math.toRadians(0.0)))
                    .lineTo(Vector2d(3.0, 59.0))
                    .build()
            robot.followTrajectory(trajectory2)

            get<WobbleGoalLift>().goDown()
            wait(1.0)
            get<ServoWobble>().ungrab()
            wait(1.0)

            val trajectory3 = robot.trajectoryBuilder(Pose2d(3.0, 59.0, Math.toRadians(0.0)))
                    .lineTo(Vector2d(-21.0, 57.0))
                    .build()
            robot.followTrajectory(trajectory3)

            get<WobbleGoalLift>().goBack()
            wait(11.0)

            val trajectory4 = robot.trajectoryBuilder(Pose2d(-21.0, 57.0, Math.toRadians(0.0)))
                    .lineTo(Vector2d(4.0, 36.0))
                    .build()
            robot.followTrajectory(trajectory4)

        }

        else if(nrRings == Recognition.NrRings.ONE){

            wait(3.0)

            val trajectory1 = robot.trajectoryBuilder(startPose)
                    .lineTo(Vector2d(-9.0, 52.0))
                    .build()
            robot.followTrajectory(trajectory1)
            robot.turn(Math.toRadians(165.0))
            get<MotorThrowerModule>().setPower(0.64)
            wait(1.75)

            for (i in 1..4) {
                wait(0.3)
                get<ServoThrowerModule>().open()
                wait(0.2)
                get<ServoThrowerModule>().close()
            }

            robot.turn(Math.toRadians(-15.0))
            get<IntakeModule>().move(true)
            val trajectory2 = robot.trajectoryBuilder(Pose2d(-30.0, 38.0, Math.toRadians(180.0)))
                    .lineTo(Vector2d(-14.0, 36.0))
                    .build()
            robot.followTrajectory(trajectory2)
            get<MotorThrowerModule>().setPower(0.64)
            robot.turn(Math.toRadians(9.0))
            wait(2.0)
            get<ServoThrowerModule>().open()
            get<IntakeModule>().stop()
            for(i in 1..2) {
                wait(0.4)
                get<ServoThrowerModule>().open()
                wait(0.4)
                get<ServoThrowerModule>().close()
            }
            get<MotorThrowerModule>().setPower(0.0)
            get<WobbleGoalLift>().goMid()
            wait(2.3)

            val trajectory3 = robot.trajectoryBuilder(Pose2d(-9.0, 52.0, Math.toRadians(180.0)))
                    .lineTo(Vector2d(16.0, 44.0))
                    .build()
            robot.followTrajectory(trajectory3)
            robot.turn(Math.toRadians(-180.0))
            get<ServoWobble>().ungrab()
            wait(1.0)
            get<WobbleGoalLift>().goBack()
            wait(1.0)
            get<ServoWobble>().grab()
            wait(1.0)
            val trajectory4 = robot.trajectoryBuilder(Pose2d(16.0, 44.0, Math.toRadians(0.0)))
                    .lineTo(Vector2d(12.0, 54.0))
                    .build()
            robot.followTrajectory(trajectory4)
        }

        else if(nrRings == Recognition.NrRings.FOUR){
            val trajectory1 = robot.trajectoryBuilder(startPose)
                    .lineTo(Vector2d(-9.0, 52.0))
                    .build()
            robot.followTrajectory(trajectory1)
            robot.turn(Math.toRadians(165.0))
            get<MotorThrowerModule>().setPower(0.65)
            wait(1.75)

            for (i in 1..4) {
                wait(0.3)
                get<ServoThrowerModule>().open()
                wait(0.2)
                get<ServoThrowerModule>().close()
            }

            get<MotorThrowerModule>().setPower(0.0)
            robot.turn(Math.toRadians(-150.0))
            val trajectory2 = robot.trajectoryBuilder(Pose2d(-9.0, -52.0, Math.toRadians(0.0)))
                    .lineTo(Vector2d(44.0, 55.0))
                    .build()
            robot.followTrajectory(trajectory2)
            get<WobbleGoalLift>().goDown()
            wait(1.0)
            get<ServoWobble>().ungrab()
            wait(1.0)
            get<WobbleGoalLift>().goBack()


            val trajectory3 = robot.trajectoryBuilder(Pose2d(44.0, 55.0, Math.toRadians(.0)))
                    .lineToLinearHeading(Pose2d(-18.0, 34.0, Math.toRadians(180.0)))
                    .build()

            get<IntakeModule>().move(true)
            robot.followTrajectory(trajectory3)

            get<MotorThrowerModule>().setPower(0.65)
            wait(1.5)

            val trajectory4 = robot.trajectoryBuilder(Pose2d(-18.0, 34.0, Math.toRadians(180.0)))
                    .lineTo(Vector2d(-30.0,38.0 ))
                    .build()
            robot.followTrajectory(trajectory4)

            val trajectory6 = robot.trajectoryBuilder(Pose2d(-30.0, 38.0, Math.toRadians(180.0)))
                    .lineTo(Vector2d(-33.0,38.0 ))
                    .build()

            robot.followTrajectory(trajectory6)

            //get<IntakeModule>().stop()

            robot.turn(Math.toRadians(9.0))

            for(i in 1..3){
                get<ServoThrowerModule>().open()
                wait(0.2 )
                get<ServoThrowerModule>().close()
                wait(0.3)
            }

            val trajectory7 = robot.trajectoryBuilder(Pose2d(-33.0, 38.0, Math.toRadians(180.0)))
                    .lineTo(Vector2d(-37.0,38.0 ))
                    .build()

            robot.followTrajectory(trajectory7)

            get<ServoThrowerModule>().open()
            wait(0.2 )
            get<ServoThrowerModule>().close()
            wait(0.3)

            val trajectory5 = robot.trajectoryBuilder(Pose2d(-37.0, 38.0, Math.toRadians(180.0)))
                    .lineTo(Vector2d(9.0,55.0 ))
                    .build()
            robot.followTrajectory(trajectory5)





        }

    }
}
