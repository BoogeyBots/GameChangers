package org.firstinspires.ftc.teamcode

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.modules.*

typealias Mecanum = SampleMecanumDrive

class Robot(val modules: Set<RobotModule>) {

    inline fun <reified T: RobotModule> get(): T = modules.first { x -> x is T } as T
}