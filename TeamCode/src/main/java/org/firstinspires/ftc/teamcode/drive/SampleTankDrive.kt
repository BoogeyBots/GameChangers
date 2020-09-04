package org.firstinspires.ftc.teamcode.drive

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.drive.TankDrive
import com.acmerobotics.roadrunner.followers.TankPIDVAFollower
import com.acmerobotics.roadrunner.followers.TrajectoryFollower
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.profile.MotionProfile
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.TankConstraints
import com.acmerobotics.roadrunner.util.NanoClock
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import org.firstinspires.ftc.teamcode.util.DashboardUtil
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil
import java.util.*

/*
 * Simple tank drive hardware implementation for REV hardware.
 */
@Config
class SampleTankDrive(hardwareMap: HardwareMap) : TankDrive(DriveConstants.kV, DriveConstants.kA, DriveConstants.kStatic, DriveConstants.TRACK_WIDTH) {
    enum class Mode {
        IDLE, TURN, FOLLOW_TRAJECTORY
    }

    private val dashboard: FtcDashboard
    private val clock: NanoClock
    private var mode: Mode
    private val turnController: PIDFController
    private var turnProfile: MotionProfile? = null
    private var turnStart = 0.0
    private val constraints: DriveConstraints
    private val follower: TrajectoryFollower
    private val poseHistory: MutableList<Pose2d>
    private val motors: List<DcMotorEx>
    private val leftMotors: List<DcMotorEx>
    private val rightMotors: List<DcMotorEx>
    private val imu: BNO055IMU
    fun trajectoryBuilder(startPose: Pose2d?): TrajectoryBuilder {
        return TrajectoryBuilder(startPose, constraints)
    }

    fun trajectoryBuilder(startPose: Pose2d?, reversed: Boolean): TrajectoryBuilder {
        return TrajectoryBuilder(startPose!!, reversed, constraints)
    }

    fun trajectoryBuilder(startPose: Pose2d?, startHeading: Double): TrajectoryBuilder {
        return TrajectoryBuilder(startPose!!, startHeading, constraints)
    }

    fun turnAsync(angle: Double) {
        val heading = poseEstimate.heading
        turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                MotionState(heading, 0, 0, 0),
                MotionState(heading + angle, 0, 0, 0),
                constraints.maxAngVel,
                constraints.maxAngAccel,
                constraints.maxAngJerk
        )
        turnStart = clock.seconds()
        mode = Mode.TURN
    }

    fun turn(angle: Double) {
        turnAsync(angle)
        waitForIdle()
    }

    fun followTrajectoryAsync(trajectory: Trajectory?) {
        poseHistory.clear()
        follower.followTrajectory(trajectory!!)
        mode = Mode.FOLLOW_TRAJECTORY
    }

    fun followTrajectory(trajectory: Trajectory?) {
        followTrajectoryAsync(trajectory)
        waitForIdle()
    }

    val lastError: Pose2d
        get() {
            return when (mode) {
                Mode.FOLLOW_TRAJECTORY -> follower.lastError
                Mode.TURN -> Pose2d(0, 0, turnController.lastError)
                Mode.IDLE -> Pose2d()
            }
            throw AssertionError()
        }

    fun update() {
        updatePoseEstimate()
        val currentPose = poseEstimate
        val lastError = lastError
        poseHistory.add(currentPose)
        val packet = TelemetryPacket()
        val fieldOverlay = packet.fieldOverlay()
        packet.put("mode", mode)
        packet.put("x", currentPose.x)
        packet.put("y", currentPose.y)
        packet.put("heading", currentPose.heading)
        packet.put("xError", lastError.x)
        packet.put("yError", lastError.y)
        packet.put("headingError", lastError.heading)
        when (mode) {
            Mode.IDLE -> {
            }
            Mode.TURN -> {
                val t = clock.seconds() - turnStart
                val targetState = turnProfile!![t]
                turnController.targetPosition = targetState.x
                val correction = turnController.update(currentPose.heading)
                val targetOmega = targetState.v
                val targetAlpha = targetState.a
                setDriveSignal(DriveSignal(Pose2d(
                        0, 0, targetOmega + correction
                ), Pose2d(
                        0, 0, targetAlpha
                )))
                if (t >= turnProfile!!.duration()) {
                    mode = Mode.IDLE
                    setDriveSignal(DriveSignal())
                }
            }
            Mode.FOLLOW_TRAJECTORY -> {
                setDriveSignal(follower.update(currentPose))
                val trajectory = follower.trajectory
                fieldOverlay.setStrokeWidth(1)
                fieldOverlay.setStroke("4CAF50")
                drawSampledPath(fieldOverlay, trajectory.path)
                val t = follower.elapsedTime()
                DashboardUtil.drawRobot(fieldOverlay, trajectory[t])
                fieldOverlay.setStroke("#3F51B5")
                DashboardUtil.drawPoseHistory(fieldOverlay, poseHistory)
                DashboardUtil.drawRobot(fieldOverlay, currentPose)
                if (!follower.isFollowing()) {
                    mode = Mode.IDLE
                    setDriveSignal(DriveSignal())
                }
            }
        }
        dashboard.sendTelemetryPacket(packet)
    }

    fun waitForIdle() {
        while (!Thread.currentThread().isInterrupted && isBusy) {
            update()
        }
    }

    val isBusy: Boolean
        get() = mode != Mode.IDLE

    fun setMode(runMode: RunMode?) {
        for (motor in motors) {
            motor.mode = runMode
        }
    }

    fun setZeroPowerBehavior(zeroPowerBehavior: ZeroPowerBehavior?) {
        for (motor in motors) {
            motor.zeroPowerBehavior = zeroPowerBehavior
        }
    }

    fun getPIDCoefficients(runMode: RunMode?): PIDCoefficients {
        val coefficients = leftMotors[0].getPIDFCoefficients(runMode)
        return PIDCoefficients(coefficients.p, coefficients.i, coefficients.d)
    }

    fun setPIDCoefficients(runMode: RunMode?, coefficients: PIDCoefficients?) {
        for (motor in motors) {
            motor.setPIDFCoefficients(runMode, PIDFCoefficients(
                    coefficients!!.kP, coefficients.kI, coefficients.kD, DriveConstants.getMotorVelocityF()
            ))
        }
    }

    override fun getWheelPositions(): List<Double> {
        var leftSum = 0.0
        var rightSum = 0.0
        for (leftMotor in leftMotors) {
            leftSum += DriveConstants.encoderTicksToInches(leftMotor.currentPosition.toDouble())
        }
        for (rightMotor in rightMotors) {
            rightSum += DriveConstants.encoderTicksToInches(rightMotor.currentPosition.toDouble())
        }
        return Arrays.asList(leftSum / leftMotors.size, rightSum / rightMotors.size)
    }

    override fun getWheelVelocities(): List<Double>? {
        var leftSum = 0.0
        var rightSum = 0.0
        for (leftMotor in leftMotors) {
            leftSum += DriveConstants.encoderTicksToInches(leftMotor.velocity)
        }
        for (rightMotor in rightMotors) {
            rightSum += DriveConstants.encoderTicksToInches(rightMotor.velocity)
        }
        return Arrays.asList(leftSum / leftMotors.size, rightSum / rightMotors.size)
    }

    override fun setMotorPowers(v: Double, v1: Double) {
        for (leftMotor in leftMotors) {
            leftMotor.power = v
        }
        for (rightMotor in rightMotors) {
            rightMotor.power = v1
        }
    }

    override val rawExternalHeading: Double
        get() = imu.angularOrientation.firstAngle.toDouble()

    companion object {
        var AXIAL_PID = PIDCoefficients(0, 0, 0)
        var CROSS_TRACK_PID = PIDCoefficients(0, 0, 0)
        var HEADING_PID = PIDCoefficients(0, 0, 0)
    }

    init {
        dashboard = FtcDashboard.getInstance()
        dashboard.telemetryTransmissionInterval = 25
        clock = NanoClock.system()
        mode = Mode.IDLE
        turnController = PIDFController(HEADING_PID)
        turnController.setInputBounds(0.0, 2 * Math.PI)
        constraints = TankConstraints(DriveConstants.BASE_CONSTRAINTS, DriveConstants.TRACK_WIDTH)
        follower = TankPIDVAFollower(AXIAL_PID, CROSS_TRACK_PID,
                Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5)
        poseHistory = ArrayList()
        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap)
        for (module in hardwareMap.getAll(LynxModule::class.java)) {
            module.bulkCachingMode = LynxModule.BulkCachingMode.AUTO
        }

        // TODO: adjust the names of the following hardware devices to match your configuration
        imu = hardwareMap.get(BNO055IMU::class.java, "imu")
        val parameters = BNO055IMU.Parameters()
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS
        imu.initialize(parameters)

        // TODO: if your hub is mounted vertically, remap the IMU axes so that the z-axis points
        // upward (normal to the floor) using a command like the following:
        // BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);

        // add/remove motors depending on your robot (e.g., 6WD)
        val leftFront = hardwareMap.get(DcMotorEx::class.java, "leftFront")
        val leftRear = hardwareMap.get(DcMotorEx::class.java, "leftRear")
        val rightRear = hardwareMap.get(DcMotorEx::class.java, "rightRear")
        val rightFront = hardwareMap.get(DcMotorEx::class.java, "rightFront")
        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront)
        leftMotors = Arrays.asList(leftFront, leftRear)
        rightMotors = Arrays.asList(rightFront, rightRear)
        for (motor in motors) {
            val motorConfigurationType = motor.motorType.clone()
            motorConfigurationType.achieveableMaxRPMFraction = 1.0
            motor.motorType = motorConfigurationType
        }
        if (DriveConstants.RUN_USING_ENCODER) {
            setMode(RunMode.RUN_USING_ENCODER)
        }
        setZeroPowerBehavior(ZeroPowerBehavior.BRAKE)
        if (DriveConstants.RUN_USING_ENCODER && DriveConstants.MOTOR_VELO_PID != null) {
            setPIDCoefficients(RunMode.RUN_USING_ENCODER, DriveConstants.MOTOR_VELO_PID)
        }

        // TODO: reverse any motors using DcMotor.setDirection()

        // TODO: if desired, use setLocalizer() to change the localization method
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));
    }
}