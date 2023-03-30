package teamcode.v1

import com.asiankoala.koawalib.math.Pose
import com.asiankoala.koawalib.subsystem.drive.KMecanumOdoDrive
import teamcode.v1.constants.ArmConstants
import teamcode.v1.constants.Strategy
import teamcode.v1.subsystems.*

class Robot(startPose: Pose) {
    var strat = Strategy.HIGH

    val hardware = Hardware(startPose)

    val drive = KMecanumOdoDrive(
        hardware.fl,
        hardware.bl,
        hardware.br,
        hardware.fr,
        hardware.odometry,
        true
    )

    val arm = Arm(hardware.armMotor, hardware.limitSwitch)
    val claw = Claw(hardware.clawServo)
    val guide = Guide(hardware.guideServo, hardware.distanceSensor, hardware.distanceSensor2)
    val whacker = Whacker(hardware.whackerServo)
    val lift = Lift(hardware.liftLeadMotor, hardware.liftSecondMotor)

    var isLifted = false

    var isStacking = false
    var stack = 5
    val stackHeight get() = (stack - 1) - 0.5

    init {
        arm.setPos(ArmConstants.groundPos)
        lift.setPos(0.0)
    }
}