package org.firstinspires.ftc.teamcode.robot.teleop

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Robot
import org.firstinspires.ftc.teamcode.bbopmode.BBLinearOpMode
import org.firstinspires.ftc.teamcode.bbopmode.get
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.modules.*

@TeleOp()
@Disabled
class TeleOpTest : BBLinearOpMode(){
    override val modules = Robot(setOf(MotorThrowerModule(this), ServoThrowerModule(this), IntakeModule(this), WobbleGoalLift(this)))

    override fun runOpMode() {
        val drive = SampleMecanumDrive(hardwareMap)
        
        var auto = false

        modules.modules.forEach(){
            it.init()
        }


        waitForStart()

        while (!isStopRequested) {

            if (gamepad1.right_trigger > 0.0) {
                forwardMovement = gamepad1.right_trigger.toDouble()
            } else if (gamepad1.left_trigger > 0.0) {
                forwardMovement = -gamepad1.left_trigger.toDouble()
            } else {
                forwardMovement = .0
            }
            drive.setWeightedDrivePower(
                    Pose2d(
                            forwardMovement,
                            (-gamepad1.left_stick_x).toDouble(),
                            (-gamepad1.right_stick_x).toDouble()
                    )
            )

            drive.update()

            if(gamepad1.right_bumper){
                get<WobbleGoalLift>().goUp()
            }
            if(gamepad1.left_bumper){
                get<WobbleGoalLift>().goDown()
            }

            if(gamepad1.b){
                get<WobbleGoalModule>().move_endgame()
            }



            if (gamepad2.x){
                get<MotorThrowerModule>().setPower(0.75)
            }
            else if(gamepad2.y){
                get<MotorThrowerModule>().setPower(0.70)
            }
            else if(gamepad2.a){
                get<MotorThrowerModule>().setPower(0.73)
            }
            else{
                get<MotorThrowerModule>().setPower(0.0)
            }

            if (gamepad2.left_bumper) {
                for (i in 1..3) {
                    drive.setWeightedDrivePower(Pose2d(0.0,0.0,0.0))
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

            if (gamepad2.left_bumper){
                get<ServoThrowerModule>().close()
            }
            else if(gamepad2.right_bumper){
                get<ServoThrowerModule>().open()
            }


            if(gamepad2.dpad_down){
                get<IntakeModule>().move(true)
            }
            else if(gamepad2.dpad_up){
                get<IntakeModule>().move(false)
            }
            else{
                get<IntakeModule>().stop()
            }

            //telemetry.addData("Putere Teoretica: ", motorPower)
            telemetry.update()

        }
    }

    companion object{
        var timeElapsed = ElapsedTime()
        var forwardMovement: Double = 0.0
    }
}