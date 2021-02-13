package org.firstinspires.ftc.teamcode.robot.autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.teamcode.Mecanum
import org.firstinspires.ftc.teamcode.Robot
import org.firstinspires.ftc.teamcode.bbopmode.BBLinearOpMode
import org.firstinspires.ftc.teamcode.bbopmode.get
import org.firstinspires.ftc.teamcode.modules.MotorThrowerModule
import org.firstinspires.ftc.teamcode.modules.Recognition
import org.firstinspires.ftc.teamcode.modules.ServoThrowerModule
import org.firstinspires.ftc.teamcode.modules.WobbleGoalModule

@Disabled
@Autonomous()
class ShootTest : BBLinearOpMode() {

    override val modules: Robot = Robot(setOf(WobbleGoalModule(this, inAuto = true), ServoThrowerModule(this), MotorThrowerModule(this), Recognition(this)))

    var i: Int = 0

    override fun runOpMode() {
        val robot = Mecanum(hardwareMap)
        modules.modules.forEach(){
            it.init()
        }

        wait(2.0)
        val nrRings = get<Recognition>().recognizeRings()

        i = when (nrRings){
            Recognition.NrRings.FOUR -> 3
            Recognition.NrRings.ONE -> 2
            Recognition.NrRings.ZERO -> 1
        }

        waitForStart()


        wait(0.6)

        for(i in 1..i){
            get<ServoThrowerModule>().open()
            wait(0.2 )
            get<ServoThrowerModule>().close()
            wait(0.3)
        }
    }
}