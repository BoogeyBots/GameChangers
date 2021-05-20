package org.firstinspires.ftc.teamcode.robot.teleop

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Robot
import org.firstinspires.ftc.teamcode.bbopmode.BBLinearOpMode
import org.firstinspires.ftc.teamcode.bbopmode.get
import org.firstinspires.ftc.teamcode.drive.DriveConstants
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.modules.IntakeModule
import org.firstinspires.ftc.teamcode.modules.MotorThrowerModule
import org.firstinspires.ftc.teamcode.modules.ServoThrowerModule
import org.firstinspires.ftc.teamcode.modules.WobbleGoalModule

@TeleOp()
class TeleOpEndGameStanga : BBLinearOpMode(){
    override val modules = Robot(setOf(WobbleGoalModule(this, inAuto = false), MotorThrowerModule(this), ServoThrowerModule(this), IntakeModule(this)))

    enum class Mode {
        DRIVER_CONTROL, AUTOMATIC_CONTROL
    }

    var currentMode = Mode.DRIVER_CONTROL

    override fun runOpMode() {
        val drive = SampleMecanumDrive(hardwareMap)

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)

        drive.poseEstimate = Pose2d(0.0,0.0,0.0)

        modules.modules.forEach(){
            it.init()
        }

        waitForStart()

        while (!isStopRequested) {

            when(currentMode) {

                Mode.DRIVER_CONTROL -> {
                    forwardMovement = when {
                        gamepad1.right_trigger > 0.0 -> {
                            gamepad1.right_trigger.toDouble()
                        }
                        gamepad1.left_trigger > 0.0 -> {
                            -gamepad1.left_trigger.toDouble()
                        }
                        else -> {
                            .0
                        }
                    }
                    drive.setWeightedDrivePower(
                            Pose2d(
                                    forwardMovement,
                                    (-gamepad1.left_stick_x).toDouble(),
                                    (-gamepad1.right_stick_x).toDouble()
                            )
                    )


                    if (gamepad1.right_bumper) {
                        get<WobbleGoalModule>().move_vertically()
                    }
                    if (gamepad1.left_bumper) {
                        get<WobbleGoalModule>().move_close()
                    }
                    if (gamepad1.b) {
                        get<WobbleGoalModule>().move_endgame()
                    }

                    when {
                        gamepad2.x -> {
                            get<MotorThrowerModule>().setPower(0.75)
                        }
                        gamepad2.y -> {
                            get<MotorThrowerModule>().setPower(0.70)
                        }
                        else -> {
                            get<MotorThrowerModule>().setPower(0.0)
                        }
                    }

                    if (gamepad2.left_bumper) {
                        for (i in 1..3) {
                            drive.setWeightedDrivePower(Pose2d(0.0, 0.0, 0.0))
                            get<ServoThrowerModule>().open()
                            wait(0.2)
                            get<ServoThrowerModule>().close()
                            wait(0.2)
                        }
                    }

                    if (gamepad2.right_bumper) {
                        get<ServoThrowerModule>().open()
                        wait(0.2)
                        get<ServoThrowerModule>().close()
                        wait(0.2)
                    }

                    if (gamepad2.left_bumper) {
                        get<ServoThrowerModule>().close()
                    } else if (gamepad2.right_bumper) {
                        get<ServoThrowerModule>().open()
                    }


                    when {
                        gamepad2.dpad_down -> {
                            get<IntakeModule>().move(true)
                        }
                        gamepad2.dpad_up -> {
                            get<IntakeModule>().move(false)
                        }
                        else -> {
                            get<IntakeModule>().stop()
                        }
                    }

                    if (gamepad1.a) {
                        get<MotorThrowerModule>().setPower(.65)

                        drive.poseEstimate = Pose2d(0.0,0.0)
                        val traj1 = drive.trajectoryBuilder(Pose2d(0.0,0.0))
                                .lineTo(Vector2d(0.0, 17.7) )
                                .build()

                        drive.followTrajectory(traj1)

                        wait(.5)

                        get<ServoThrowerModule>().open()
                        wait(.2)
                        get<ServoThrowerModule>().close()
                        wait(.3)

                        drive.poseEstimate = Pose2d(0.0,0.0)
                        val traj2 = drive.trajectoryBuilder(Pose2d(0.0,0.0))
                                .lineTo(Vector2d(0.0, 7.40))
                                .build()

                        drive.followTrajectory(traj2)

                        wait(1.5)
                        get<ServoThrowerModule>().open()
                        wait(.2)
                        get<ServoThrowerModule>().close()
                        wait(.3)

                        drive.poseEstimate = Pose2d(0.0,0.0)
                        val traj3 = drive.trajectoryBuilder(Pose2d(0.0,0.0), )
                                .lineTo(Vector2d(0.0, 7.40))
                                .build()

                        drive.followTrajectory(traj3)


                        wait(1.5)
                        get<ServoThrowerModule>().open()
                        wait(.2)
                        get<ServoThrowerModule>().close()
                        wait(.3)
                        currentMode = Mode.AUTOMATIC_CONTROL
                    }
                    drive.update()

                    //telemetry.addData("Putere Teoretica: ", motorPower)

                }
                Mode.AUTOMATIC_CONTROL -> {
                    if (!drive.isBusy) {
                        currentMode = Mode.DRIVER_CONTROL
                        drive.update()
                    }

                }
            }
            drive.update()
        }
    }

    companion object{
        var timeElapsed = ElapsedTime()
        var forwardMovement: Double = 0.0
    }
}