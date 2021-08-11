package org.firstinspires.ftc.teamcode.robot.autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.Mecanum
import org.firstinspires.ftc.teamcode.Robot
import org.firstinspires.ftc.teamcode.bbopmode.BBLinearOpMode
import org.firstinspires.ftc.teamcode.bbopmode.get
import org.firstinspires.ftc.teamcode.modules.MotorThrowerModule
import org.firstinspires.ftc.teamcode.modules.Recognition
import org.firstinspires.ftc.teamcode.modules.ServoThrowerModule
import org.firstinspires.ftc.teamcode.modules.WobbleGoalModule
import org.firstinspires.ftc.teamcode.vision.TensorFlowObjectDetection
import org.opencv.core.Mat

@Autonomous(name = "ROSU_OneWobbleGoal")
class ROSU_OneWobbleGoal : BBLinearOpMode(){

    override val modules: Robot = Robot(setOf(WobbleGoalModule(this, inAuto = true), ServoThrowerModule(this), MotorThrowerModule(this), Recognition(this)))


    override fun runOpMode() {
        val robot = Mecanum(hardwareMap)
        modules.modules.forEach(){
            it.init()
        }

        val nrRings = get<Recognition>().recognizeRings()

        val startPose = Pose2d(-63.0, -43.7)
        robot.poseEstimate = startPose

        waitForStart()

        if(nrRings == Recognition.NrRings.ONE) {
            val trajectory1 = robot.trajectoryBuilder(startPose)
                    .splineTo(Vector2d(-20.0, -52.0), Math.toRadians(0.0))
                    .splineTo(Vector2d(27.0, -42.0), Math.toRadians(0.0))

                    .addDisplacementMarker {
                        get<WobbleGoalModule>().move_auto()
                        wait(.3)
                    }
                    .addDisplacementMarker {
                        get<WobbleGoalModule>().move_close()
                    }
                    .build()

            val trajectory2 = robot.trajectoryBuilder(Pose2d(15.0, -42.0, 0.0))
                    .lineTo(Vector2d(-4.0, -36.0))
                    .build()

            val trajectory3 = robot.trajectoryBuilder(Pose2d(-4.0, -36.0), Math.toRadians(180.0))
                    .lineTo(Vector2d(10.0, -50.0))
                    .build()
            
            robot.followTrajectory(trajectory1)
            robot.followTrajectory(trajectory2)
            robot.turn(Math.toRadians(180.0))

            get<MotorThrowerModule>().setPower(0.75)
            wait(0.75)

            for(i in 1..3){
                get<ServoThrowerModule>().open()
                wait(0.2 )
                get<ServoThrowerModule>().close()
                wait(0.3)
            }

            get<MotorThrowerModule>().setPower(0.0)

            robot.followTrajectory(trajectory3)
            
        }

        if(nrRings == Recognition.NrRings.ZERO){
            val trajectory1 = robot.trajectoryBuilder(startPose)
                    .lineTo(Vector2d(-0.7, -62.0))
                    .addDisplacementMarker {
                        get<WobbleGoalModule>().move_auto()
                        wait(.3)
                    }
                    .addDisplacementMarker {
                        get<WobbleGoalModule>().move_close()
                    }
                    .build()

            val trajectory2 = robot.trajectoryBuilder(robot.poseEstimate)
                    .lineTo(Vector2d(-3.0, -36.0))
                    .build()

            val trajectory3 = robot.trajectoryBuilder(trajectory2.end(), Math.toRadians(180.0))
                    .lineTo(Vector2d(10.0, -50.0))
                    .build()
            
            robot.followTrajectory(trajectory1)
            wait(.5)
            robot.followTrajectory(trajectory2)
            robot.turn(Math.toRadians(180.0))
            get<MotorThrowerModule>().setPower(0.75)
            wait(0.75)

            for(i in 1..3){
                get<ServoThrowerModule>().open()
                wait(0.2 )
                get<ServoThrowerModule>().close()
                wait(0.3)
            }

            get<MotorThrowerModule>().setPower(0.0)

            robot.followTrajectory(trajectory3)
            
        }

        if(nrRings == Recognition.NrRings.FOUR) {
            val trajectory1 = robot.trajectoryBuilder(startPose)
                    .splineTo(Vector2d(-25.0, -55.0), Math.toRadians(0.0))
                    .splineTo(Vector2d(50.0, -60.5), Math.toRadians(0.0))

                    .addDisplacementMarker {
                        get<WobbleGoalModule>().move_auto()
                        wait(.3)
                    }
                    .addDisplacementMarker {
                        get<WobbleGoalModule>().move_close()
                    }
                    .build()

            val trajectory2 = robot.trajectoryBuilder(Pose2d(52.0, -60.5, 0.0))
                    .lineTo(Vector2d(-4.0, -36.0))
                    .build()
/*
            val trajectory3 = robot.trajectoryBuilder(trajectory2.end(), Math.toRadians(180.0))
                    .lineTo(Vector2d(10 .0, -50.0))
                    ț?val trajectory3 = robot.trajectoryBuilder(Pose2d(-4.0, -36.0, Math.toRadians(180.0)))
            .lineTo(Vector2d(6.0, -12.0))
                    .build()?mINJ.UBGM V, lk ?IBUYNHJXCx
*/
            robot.followTrajectory(trajectory1)
            robot.followTrajectory(trajectory2)
            robot.turn(Math.toRadians(180.0))
            get<MotorThrowerModule>().setPower(0.75)
            wait(0.75)

            for(i in 1..3){
                get<ServoThrowerModule>().open()
                wait(0.2 )
                get<ServoThrowerModule>().close()
                wait(0.3)
            }

            get<MotorThrowerModule>().setPower(0.0)

  //          robot.followTrajectory(trajectory3)
        }


    }


}