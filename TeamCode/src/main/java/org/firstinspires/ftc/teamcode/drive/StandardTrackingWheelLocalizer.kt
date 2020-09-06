package org.firstinspires.ftc.teamcode.drive

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.util.Encoder
import java.util.*

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
class StandardTrackingWheelLocalizer(hardwareMap: HardwareMap) : ThreeTrackingWheelLocalizer(Arrays.asList(
        Pose2d(0.0, LATERAL_DISTANCE / 2, 0.0),  // left
        Pose2d(0.0, -LATERAL_DISTANCE / 2, 0.0),  // right
        Pose2d(FORWARD_OFFSET, 0.0, Math.toRadians(90.0)) // front
)) {
    private val leftEncoder: Encoder
    private val rightEncoder: Encoder
    private val frontEncoder: Encoder
    override fun getWheelPositions(): List<Double> {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.currentPosition.toDouble()),
                encoderTicksToInches(rightEncoder.currentPosition.toDouble()),
                encoderTicksToInches(frontEncoder.currentPosition.toDouble())
        )
    }

    override fun getWheelVelocities(): List<Double> {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.rawVelocity),
                encoderTicksToInches(rightEncoder.rawVelocity),
                encoderTicksToInches(frontEncoder.rawVelocity)
        )
    }

    companion object {
        var TICKS_PER_REV = 0.0
        var WHEEL_RADIUS = 2.0 // in
        var GEAR_RATIO = 1.0 // output (wheel) speed / input (encoder) speed
        var LATERAL_DISTANCE = 10.0 // in; distance between the left and right wheels
        var FORWARD_OFFSET = 4.0 // in; offset of the lateral wheel
        fun encoderTicksToInches(ticks: Double): Double {
            return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV
        }
    }

    init {
        leftEncoder = Encoder(hardwareMap.get(DcMotorEx::class.java, "leftEncoder"))
        rightEncoder = Encoder(hardwareMap.get(DcMotorEx::class.java, "rightEncoder"))
        frontEncoder = Encoder(hardwareMap.get(DcMotorEx::class.java, "frontEncoder"))

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    }
}