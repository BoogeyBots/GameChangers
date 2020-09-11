package org.firstinspires.ftc.teamcode.robot.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Mecanum
import kotlin.math.abs


@TeleOp
open class BaseTeleOp : LinearOpMode() {

    override fun runOpMode() {
        val robot = Mecanum(null)
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        waitForStart()

        while (!isStopRequested) {
            driveRobot(robot)
        }

        telemetry.addData("x:", robot.poseEstimate.x)
        telemetry.addData("y:", robot.poseEstimate.y)

    }
    fun driveRobot(robot: Mecanum){
        val baseVel = Pose2d(
                (-gamepad1.left_stick_y).toDouble(),
                (-gamepad1.left_stick_x).toDouble(),
                (-gamepad1.right_stick_x).toDouble()
        )

        val vel: Pose2d
        vel = (if (abs(baseVel.x) + abs(baseVel.y) + abs(baseVel.heading) > 1) {
            // re-normalize the powers according to the weights
            val denom = abs(baseVel.x) +  abs(baseVel.y) +  abs(baseVel.heading)
            Pose2d(
                    baseVel.x,
                    baseVel.y,
                    baseVel.heading
            ).div(denom)
        } else baseVel) // nu stiu ce se intampla aici tbh

        robot.setDrivePower(vel)

        robot.update()
    }
}