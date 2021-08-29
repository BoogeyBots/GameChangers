package org.firstinspires.ftc.teamcode.test

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.teamcode.Robot
import org.firstinspires.ftc.teamcode.bbopmode.BBLinearOpMode
import org.firstinspires.ftc.teamcode.bbopmode.get
import org.firstinspires.ftc.teamcode.modules.ServoThrowerModule
import org.firstinspires.ftc.teamcode.modules.TestModule
@Disabled
class TestServoBackForth : BBLinearOpMode(){
    override val modules = Robot(setOf(ServoThrowerModule(this)))

    override fun runOpMode() {
        if(gamepad1.a){
            for(i in 1..3){
                get<ServoThrowerModule>().open()
                wait(0.2)
                get<ServoThrowerModule>().close()
                wait(0.2)
            }
        }

    }
}