package org.firstinspires.ftc.teamcode.robot.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.Mecanum
import kotlin.math.abs


@TeleOp(group = "drive")
open class BaseTeleOp : LinearOpMode() {

    enum class ROBOT_MODE {
        CONTROLLED,
        AUTO
    }

    private var currentROBOT_MODE = ROBOT_MODE.CONTROLLED

    override fun runOpMode() {
        val robot = Mecanum(hardwareMap)
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        robot.poseEstimate = Pose2d(0.0,0.0,0.0)
        waitForStart()

        robot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        robot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)

        while (!isStopRequested) {
            driveRobot(robot)
        }


    }
    fun driveRobot(robot: Mecanum){
        if(gamepad1.a){
            currentROBOT_MODE = ROBOT_MODE.AUTO
        }

        else if(gamepad1.b){
            currentROBOT_MODE = ROBOT_MODE.CONTROLLED
        }

        if(currentROBOT_MODE == ROBOT_MODE.CONTROLLED){
            robot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
            val baseVel = Pose2d(
                    (-gamepad1.left_stick_y).toDouble(),
                    (-gamepad1.left_stick_x).toDouble(),
                    (-gamepad1.right_stick_x).toDouble()
            )

            val vel: Pose2d
            vel = (if (abs(baseVel.x) + abs(baseVel.y) + abs(baseVel.heading) > 1)
            {
                // re-normalize the powers according to the weights
                val denom = abs(baseVel.x) +  abs(baseVel.y) +  abs(baseVel.heading)
                Pose2d(
                        baseVel.x,
                        baseVel.y,
                        baseVel.heading
                ).div(denom)
            }
            else baseVel
            ) // nu stiu ce se intampla aici tbh

            robot.setDrivePower(vel)

            robot.update()

            val poseEstimate: Pose2d = robot.poseEstimate
            telemetry.addData("x", poseEstimate.x)
            telemetry.addData("y", poseEstimate.y)
            telemetry.addData("heading", poseEstimate.heading)
            telemetry.update()
        }

        else {
            robot.setMode(DcMotor.RunMode.RUN_USING_ENCODER)
            robot.followTrajectory(
                    robot.trajectoryBuilder(robot.poseEstimate)
                    .splineTo(Vector2d(40.0,0.0), 0.0)
                    .build())
            currentROBOT_MODE = ROBOT_MODE.CONTROLLED
        }
    }
}