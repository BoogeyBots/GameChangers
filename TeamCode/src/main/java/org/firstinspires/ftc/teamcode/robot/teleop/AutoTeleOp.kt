package org.firstinspires.ftc.teamcode.robot.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.Mecanum
import org.firstinspires.ftc.teamcode.Robot
import org.firstinspires.ftc.teamcode.bbopmode.BBLinearOpMode
import org.firstinspires.ftc.teamcode.modules.TestModule
import org.firstinspires.ftc.teamcode.util.PoseStorage
import kotlin.math.abs


@TeleOp(group = "drive")
open class AutoTeleOp : BBLinearOpMode() {

    enum class ROBOTMODE {
        CONTROLLED,
        AUTO
    }

    override val modules = Robot(setOf(TestModule(this)))

    private var robotMode = ROBOTMODE.CONTROLLED

    override fun runOpMode() {
        val robot = Mecanum(hardwareMap)

        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        robot.poseEstimate = Pose2d(0.0,0.0,0.0)
        waitForStart()

        robot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        robot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)

        while (!isStopRequested) {
            driveRobot(robot, false)
        }

        // Store the last position of the robot during teleop
        PoseStorage.currentPose = robot.poseEstimate
    }
    fun driveRobot(robot: Mecanum, noMode: Boolean){
        if(!noMode) {
            if (gamepad1.a) {
                robotMode = ROBOTMODE.AUTO
            } else if (gamepad1.b) {
                robotMode = ROBOTMODE.CONTROLLED
            }
        }
        else {
            robotMode = ROBOTMODE.CONTROLLED
        }
        if(robotMode == ROBOTMODE.CONTROLLED){
            robot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
            val baseVel = Pose2d(
                    (-gamepad1.left_stick_y).toDouble(),
                    (-gamepad1.left_stick_x).toDouble(),
                    (-gamepad1.right_stick_x).toDouble()
            )

            val vel: Pose2d
            vel = (
                    if (abs(baseVel.x) + abs(baseVel.y) + abs(baseVel.heading) > 1)
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
                    .splineTo(Vector2d(0.0,0.0), 0.0)
                    .build())
            robotMode = ROBOTMODE.CONTROLLED
        }

    }
}