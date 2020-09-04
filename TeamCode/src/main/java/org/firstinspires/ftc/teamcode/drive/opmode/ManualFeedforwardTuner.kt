package org.firstinspires.ftc.teamcode.drive.opmode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.kinematics.Kinematics.calculateMotorFeedforward
import com.acmerobotics.roadrunner.profile.MotionProfile
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.acmerobotics.roadrunner.util.NanoClock
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.drive.DriveConstants
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import java.util.*

/*
 * This routine is designed to tune the open-loop feedforward coefficients. Although it may seem unnecessary,
 * tuning these coefficients is just as important as the positional parameters. Like the other
 * manual tuning routines, this op mode relies heavily upon the dashboard. To access the dashboard,
 * connect your computer to the RC's WiFi network and navigate to https://192.168.49.1:8080/dash in
 * your browser. Once you've successfully connected, start the program, and your robot will begin
 * moving forward and backward according to a motion profile. Your job is to graph the velocity
 * errors over time and adjust the feedforward coefficients. Once you've found a satisfactory set
 * of gains, add them to your drive class.
 */
@Config
@Autonomous(group = "drive")
class ManualFeedforwardTuner : LinearOpMode() {
    private val dashboard = FtcDashboard.getInstance()
    private var drive: SampleMecanumDrive? = null
    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, dashboard.telemetry)
        drive = SampleMecanumDrive(hardwareMap)
        val clock = NanoClock.system()
        telemetry.addLine("Ready!")
        telemetry.update()
        telemetry.clearAll()
        waitForStart()
        if (isStopRequested) return
        var movingForwards = true
        var activeProfile = generateProfile(true)
        var profileStart = clock.seconds()
        while (!isStopRequested) {
            // calculate and set the motor power
            val profileTime = clock.seconds() - profileStart
            if (profileTime > activeProfile.duration()) {
                // generate a new profile
                movingForwards = !movingForwards
                activeProfile = generateProfile(movingForwards)
                profileStart = clock.seconds()
            }
            val motionState = activeProfile[profileTime]
            val targetPower = calculateMotorFeedforward(motionState.v, motionState.a, DriveConstants.kV, DriveConstants.kA, DriveConstants.kStatic)
            drive!!.setDrivePower(Pose2d(targetPower, 0, 0))
            drive!!.updatePoseEstimate()

            // update telemetry
            telemetry.addData("targetVelocity", motionState.v)
            val (currentVelo) = Objects.requireNonNull(drive!!.poseVelocity, "poseVelocity() must not be null. Ensure that the getWheelVelocities() method has been overridden in your localizer.")
            telemetry.addData("poseVelocity", currentVelo)
            telemetry.addData("error", currentVelo - motionState.v)
            telemetry.update()
        }
    }

    companion object {
        var DISTANCE = 72.0 // in
        private fun generateProfile(movingForward: Boolean): MotionProfile {
            val start = MotionState(if (movingForward) 0 else DISTANCE, 0, 0, 0)
            val goal = MotionState(if (movingForward) DISTANCE else 0, 0, 0, 0)
            return MotionProfileGenerator.generateSimpleMotionProfile(start, goal,
                    DriveConstants.BASE_CONSTRAINTS.maxVel,
                    DriveConstants.BASE_CONSTRAINTS.maxAccel,
                    DriveConstants.BASE_CONSTRAINTS.maxJerk)
        }
    }
}