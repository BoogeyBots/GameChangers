package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.Hardware
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.modules.*

typealias Mecanum = SampleMecanumDrive

class Robot(val modules: Set<RobotModule>){

    inline fun <reified T: RobotModule> get(): T = modules.first { x -> x is T } as T

}