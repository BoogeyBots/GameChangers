/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
package org.firstinspires.ftc.teamcode.vision

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.opencv.core.*
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.*
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener

/*
 * This sample demonstrates a basic (but battle-tested and essentially
 * 100% accurate) method of detecting the skystone when lined up with
 * the sample regions over the first 3 stones.
 */
@Disabled
@TeleOp
class SkystoneDeterminationExample : LinearOpMode() {
    var phoneCam: OpenCvInternalCamera? = null
    var pipeline: SkystoneDeterminationPipeline? = null
    override fun runOpMode() {
        /**
         * NOTE: Many comments have been omitted from this sample for the
         * sake of conciseness. If you're just starting out with EasyOpenCv,
         * you should take a look at [InternalCamera1Example] or its
         * webcam counterpart, [WebcamExample] first.
         */
        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.packageName)
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId)
        pipeline = SkystoneDeterminationPipeline()
        phoneCam?.setPipeline(pipeline)

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        phoneCam?.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW)
        phoneCam?.openCameraDeviceAsync(AsyncCameraOpenListener { phoneCam?.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT) })
        waitForStart()
        while (opModeIsActive()) {
            telemetry.addData("Analysis", pipeline!!.analysis)
            telemetry.update()

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50)
        }
    }

    class SkystoneDeterminationPipeline : OpenCvPipeline() {
        /*
         * An enum to define the skystone position
         */
        enum class SkystonePosition {
            LEFT, CENTER, RIGHT
        }

        /*
         * Points which actually define the sample region rectangles, derived from above values
         *
         * Example of how points A and B work to define a rectangle
         *
         *   ------------------------------------
         *   | (0,0) Point A                    |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                  Point B (70,50) |
         *   ------------------------------------
         *
         */
        var region1_pointA = Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y)
        var region1_pointB = Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT)
        var region2_pointA = Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x,
                REGION2_TOPLEFT_ANCHOR_POINT.y)
        var region2_pointB = Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT)
        var region3_pointA = Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x,
                REGION3_TOPLEFT_ANCHOR_POINT.y)
        var region3_pointB = Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT)

        /*
         * Working variables
         */
        var region1_Cb: Mat? = null
        var region2_Cb: Mat? = null
        var region3_Cb: Mat? = null
        var YCrCb = Mat()
        var Cb = Mat()
        var avg1 = 0
        var avg2 = 0
        var avg3 = 0

        /*
           * Call this from the OpMode thread to obtain the latest analysis
           */
        // Volatile since accessed by OpMode thread w/o synchronization
        @Volatile
        var analysis = SkystonePosition.LEFT
            private set

        /*
           * This function takes the RGB frame, converts to YCrCb,
           * and extracts the Cb channel to the 'Cb' variable
           */
        fun inputToCb(input: Mat?) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb)
            Core.extractChannel(YCrCb, Cb, 2)
        }

        override fun init(firstFrame: Mat) {
            /*
             * We need to call this in order to make sure the 'Cb'
             * object is initialized, so that the submats we make
             * will still be linked to it on subsequent frames. (If
             * the object were to only be initialized in processFrame,
             * then the submats would become delinked because the backing
             * buffer would be re-allocated the first time a real frame
             * was crunched)
             */
            inputToCb(firstFrame)

            /*
             * Submats are a persistent reference to a region of the parent
             * buffer. Any changes to the child affect the parent, and the.
             * reverse also holds true.
             */
            region1_Cb = Cb.submat(Rect(region1_pointA, region1_pointB))
            region2_Cb = Cb.submat(Rect(region2_pointA, region2_pointB))
            region3_Cb = Cb.submat(Rect(region3_pointA, region3_pointB))
        }

        override fun processFrame(input: Mat): Mat {
            /*
             * Overview of what we're doing:
             *
             * We first convert to YCrCb color space, from RGB color space.
             * Why do we do this? Well, in the RGB color space, chroma and
             * luma are intertwined. In YCrCb, chroma and luma are separated.
             * YCrCb is a 3-channel color space, just like RGB. YCrCb's 3 channels
             * are Y, the luma channel (which essentially just a B&W image), the
             * Cr channel, which records the difference from red, and the Cb channel,
             * which records the difference from blue. Because chroma and luma are
             * not related in YCrCb, vision code written to look for certain values
             * in the Cr/Cb channels will not be severely affected by differing
             * light intensity, since that difference would most likely just be
             * reflected in the Y channel.
             *
             * After we've converted to YCrCb, we extract just the 2nd channel, the
             * Cb channel. We do this because stones are bright yellow and contrast
             * STRONGLY on the Cb channel against everything else, including SkyStones
             * (because SkyStones have a black label).
             *
             * We then take the average pixel value of 3 different regions on that Cb
             * channel, one positioned over each stone. The brightest of the 3 regions
             * is where we assume the SkyStone to be, since the normal stones show up
             * extremely darkly.
             *
             * We also draw rectangles on the screen showing where the sample regions
             * are, as well as drawing a solid rectangle over top the sample region
             * we believe is on top of the SkyStone.
             *
             * In order for this whole process to work correctly, each sample region
             * should be positioned in the center of each of the first 3 stones, and
             * be small enough such that only the stone is sampled, and not any of the
             * surroundings.
             */

            /*
             * Get the Cb channel of the input frame after conversion to YCrCb
             */
            inputToCb(input)

            /*
             * Compute the average pixel value of each submat region. We're
             * taking the average of a single channel buffer, so the value
             * we need is at index 0. We could have also taken the average
             * pixel value of the 3-channel image, and referenced the value
             * at index 2 here.
             */avg1 = Core.mean(region1_Cb).`val`[0].toInt()
            avg2 = Core.mean(region2_Cb).`val`[0].toInt()
            avg3 = Core.mean(region3_Cb).`val`[0].toInt()

            /*
             * Draw a rectangle showing sample region 1 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */Imgproc.rectangle(
                    input,  // Buffer to draw on
                    region1_pointA,  // First point which defines the rectangle
                    region1_pointB,  // Second point which defines the rectangle
                    BLUE,  // The color the rectangle is drawn in
                    2) // Thickness of the rectangle lines

            /*
             * Draw a rectangle showing sample region 2 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */Imgproc.rectangle(
                    input,  // Buffer to draw on
                    region2_pointA,  // First point which defines the rectangle
                    region2_pointB,  // Second point which defines the rectangle
                    BLUE,  // The color the rectangle is drawn in
                    2) // Thickness of the rectangle lines

            /*
             * Draw a rectangle showing sample region 3 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */Imgproc.rectangle(
                    input,  // Buffer to draw on
                    region3_pointA,  // First point which defines the rectangle
                    region3_pointB,  // Second point which defines the rectangle
                    BLUE,  // The color the rectangle is drawn in
                    2) // Thickness of the rectangle lines


            /*
             * Find the max of the 3 averages
             */
            val maxOneTwo = Math.max(avg1, avg2)
            val max = Math.max(maxOneTwo, avg3)

            /*
             * Now that we found the max, we actually need to go and
             * figure out which sample region that value was from
             */if (max == avg1) // Was it from region 1?
            {
                analysis = SkystonePosition.LEFT // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */Imgproc.rectangle(
                    input,  // Buffer to draw on
                    region1_pointA,  // First point which defines the rectangle
                    region1_pointB,  // Second point which defines the rectangle
                    GREEN,  // The color the rectangle is drawn in
                    -1) // Negative thickness means solid fill
            } else if (max == avg2) // Was it from region 2?
            {
                analysis = SkystonePosition.CENTER // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */Imgproc.rectangle(
                    input,  // Buffer to draw on
                    region2_pointA,  // First point which defines the rectangle
                    region2_pointB,  // Second point which defines the rectangle
                    GREEN,  // The color the rectangle is drawn in
                    -1) // Negative thickness means solid fill
            } else if (max == avg3) // Was it from region 3?
            {
                analysis = SkystonePosition.RIGHT // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */Imgproc.rectangle(
                    input,  // Buffer to draw on
                    region3_pointA,  // First point which defines the rectangle
                    region3_pointB,  // Second point which defines the rectangle
                    GREEN,  // The color the rectangle is drawn in
                    -1) // Negative thickness means solid fill
            }

            /*
             * Render the 'input' buffer to the viewport. But note this is not
             * simply rendering the raw camera feed, because we called functions
             * to add some annotations to this buffer earlier up.
             */return input
        }

        companion object {
            /*
         * Some color constants
         */
            val BLUE = Scalar(0.0, 0.0, 255.0)
            val GREEN = Scalar(0.0, 255.0, 0.0)

            /*
         * The core values which define the location and size of the sample regions
         */
            val REGION1_TOPLEFT_ANCHOR_POINT = Point(109.0, 98.0)
            val REGION2_TOPLEFT_ANCHOR_POINT = Point(181.0, 98.0)
            val REGION3_TOPLEFT_ANCHOR_POINT = Point(253.0, 98.0)
            const val REGION_WIDTH = 20
            const val REGION_HEIGHT = 20
        }
    }
}