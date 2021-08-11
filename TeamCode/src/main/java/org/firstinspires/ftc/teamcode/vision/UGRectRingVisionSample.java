package org.firstinspires.ftc.teamcode.vision;

import com.arcrobotics.ftclib.vision.UGRectDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class UGRectRingVisionSample extends LinearOpMode {

    UGRectDetector UGRectDetector;

    @Override
    public void runOpMode() {
        UGRectDetector = new UGRectDetector(hardwareMap);
        UGRectDetector.init();

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            UGRectDetector.Stack stack = UGRectDetector.getStack();
            switch (stack) {
                case ZERO:
                    break;
                case ONE:
                    break;
                case FOUR:
                    break;
                default:
                    break;
            }
            telemetry.addData("Rings", stack);
        }
    }

}