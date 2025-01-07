package pedroPathing.Mittens4point0

import com.pedropathing.follower.Follower
import com.pedropathing.localization.Pose
import com.pedropathing.util.Constants
import com.pedropathing.util.Timer
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import pedroPathing.constants.FConstants
import pedroPathing.constants.LConstants
import kotlin.math.abs
import kotlin.math.max


class TeleOperatonal : LinearOpMode() {
    private val opmodeTimer: Timer? = null
    var speed: Double = 0.75

    var intakeMotorMin: Int = 0

    var intakeMotorMax: Int = 2000 //replace with empirical value for max encoder pos

    var liftingMotorMin: Int = 0

    var liftingMotorMax: Int = 3000 //replace with empirical value for max encoder pos
    var specimenMotorMin: Int = 0
    var specimenMotorPlace: Int = 1600//replace with empirical value for placing encoder pos
    var specimenMotorMax: Int = 2000 //replace with empirical value for max encoder pos
    var intakePower: Double = 1.0

    var servoIntakeUpPos: Double = 0.0 //replace with empirical value for intake starting pos

    var servoIntakeDownPos: Double = 0.0 //replace with empirical value for intake intaking pos

    var servoSlideDownPos: Double = 0.0 //replace with empirical value for dumper starting pos on slide
    var servoSlideUpPos: Double = 0.0 //replace with empirical value for dumper dumping pos on slide
    var specimenservo1OpenPos: Double = 0.0
    var specimenservo1ClosedPos: Double = 0.0
    var specimenservo2OpenPos: Double = 0.0
    var specimenservo2ClosedPos: Double = 0.0

    var servoDumperUpPos: Double = 0.0

    var servoDumperDownPos: Double = 0.0
    lateinit var servoSlide: Servo
    lateinit var servoIntake: Servo
    lateinit var servospecimen1: Servo
    lateinit var servospecimen2: Servo
    lateinit var intake1: CRServo
    lateinit var motorIntake: DcMotor
    lateinit var motorUp: DcMotor
    lateinit var motorSpecimen: DcMotor
    lateinit var servoDumper: Servo

    private var follower: Follower? = null
    private val startPose = Pose(0.0, 0.0, 0.0)


    @Throws(InterruptedException::class)
    override fun runOpMode() {
        servoDumper = hardwareMap.servo["sd"]
        servoSlide = hardwareMap.servo["ss"]
        servoIntake = hardwareMap.servo["si"]
        servospecimen1 = hardwareMap.servo["ss1"]
        servospecimen2 = hardwareMap.servo["ss2"]
        intake1 = hardwareMap.crservo["si1"]
        motorIntake = hardwareMap.dcMotor["mi"]
        motorUp = hardwareMap.dcMotor["mu"]
        motorSpecimen = hardwareMap.dcMotor["ms"]
        val motorFrontLeft = hardwareMap.dcMotor["mfl"]
        val motorFrontRight = hardwareMap.dcMotor["mfr"]
        val motorBackLeft = hardwareMap.dcMotor["mbl"]
        val motorBackRight = hardwareMap.dcMotor["mbr"]

        motorFrontLeft.direction = DcMotorSimple.Direction.REVERSE
        motorBackLeft.direction = DcMotorSimple.Direction.REVERSE
        motorUp.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motorIntake.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motorSpecimen.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motorUp.targetPosition = 0
        motorIntake.targetPosition = 0
        motorSpecimen.targetPosition = 0
        motorUp.mode = DcMotor.RunMode.RUN_TO_POSITION
        motorIntake.mode = DcMotor.RunMode.RUN_TO_POSITION
        motorSpecimen.mode = DcMotor.RunMode.RUN_TO_POSITION
        motorUp.power = 1.0
        motorIntake.power = 1.0
        motorSpecimen.power = 1.0
        servoSlide.position = servoSlideDownPos
        servoIntake.position = servoIntakeUpPos
        intake1.power = 0.0
        servoDumper.position = servoDumperDownPos
        servospecimen1.position = specimenservo1OpenPos
        servospecimen2.position = specimenservo2OpenPos
        var step = 0
        Constants.setConstants(FConstants::class.java, LConstants::class.java)
        follower = Follower(hardwareMap)
        follower!!.setStartingPose(startPose)



        while (!isStopRequested && !opModeIsActive()) {
            follower!!.startTeleopDrive()
        }
        if (isStopRequested) return
        while (opModeIsActive()) {
            waitForStart()


            /* Update Pedro to move the robot based on:
        - Forward/Backward Movement: -gamepad1.left_stick_y
        - Left/Right Movement: -gamepad1.left_stick_x
        - Turn Left/Right Movement: -gamepad1.right_stick_x
        - Robot-Centric Mode: true
        */
            follower!!.setTeleOpMovementVectors(-gamepad1.left_stick_y.toDouble()*speed, -gamepad1.left_stick_x.toDouble()*speed, -gamepad1.right_stick_x.toDouble()*speed, true)
            follower!!.update()


            /* Telemetry Outputs of our Follower */
            telemetry.addData("X", follower!!.pose.x)
            telemetry.addData("Y", follower!!.pose.y)
            telemetry.addData("Heading in Degrees", Math.toDegrees(follower!!.pose.heading))


            /* Update Telemetry to the Driver Hub */
            telemetry.update()


/*            val y = -gamepad1.left_stick_y.toDouble() // Remember, Y stick value is reversed
            val x = gamepad1.left_stick_x.toDouble() // Counteract imperfect strafing
            val rx = gamepad1.right_stick_x.toDouble()*/
            if (gamepad1.dpad_up) {
                speed = 1.0
            }
            if (gamepad1.dpad_right) {
                speed = 0.75
            }
            if (gamepad1.dpad_down) {
                speed = 0.5
            }
            if (gamepad1.dpad_left) {
                speed = 0.25
            }
 /*           val denominator = max(abs(y) + abs(x) + abs(rx), 1.0)
            val frontLeftPower = (y + x + rx) * speed / denominator
            val backLeftPower = (y - x + rx) * speed / denominator
            val frontRightPower = (y - x - rx) * speed / denominator
            val backRightPower = (y + x - rx) * speed / denominator
            motorFrontLeft.power = frontLeftPower
            motorBackLeft.power = backLeftPower
            motorFrontRight.power = frontRightPower
            motorBackRight.power = backRightPower

*/

            if (gamepad2.a){
                step = 1
            }
            if (gamepad2.b){
                step =2
            }
            if (gamepad2.x){
                step = 3
            }
            if (gamepad2.y){
                step = 4
            }
            if (gamepad2.right_bumper && step < 5){
                step = 5
            }




            when (step) {
                0 -> {
                    break;
                }
                1 -> {
                    intaking()
                }

                2 -> {
                    transferring()
                    opmodeTimer!!.resetTimer()
                    if (opmodeTimer.elapsedTime >200) {
                        lifting()
                        if (motorUp.currentPosition == liftingMotorMax - 50)
                            break
                    }
                }
                3 -> {
                    dumping()
                    opmodeTimer!!.resetTimer()
                    if (opmodeTimer.elapsedTime >200) {
                        returning()
                        if (motorUp.currentPosition == liftingMotorMin - 50)
                            break
                    }

                }
                4 -> {
                    cancellation()
                    opmodeTimer!!.resetTimer()
                    if (opmodeTimer.elapsedTime >300) {
                        break
                    }
                }

                5 -> {
                    specimenPick()
                    if (gamepad2.left_bumper){
                        step = 6
                    }
                }

                6 -> {
                    specimenLift()
                    if (gamepad2.right_bumper) step = 7
                }
                7 ->{
                    specimenPut()
                }
            }
        }
    }

    fun intaking() {
        motorIntake.targetPosition = intakeMotorMax
        motorIntake.mode = DcMotor.RunMode.RUN_TO_POSITION
        opmodeTimer!!.resetTimer()
        if (opmodeTimer.elapsedTime >200) {
            servoIntake.position = servoIntakeDownPos
            intake1.power = intakePower
        }
    }

    fun transferring() {
        servoIntake.position = servoIntakeUpPos
        motorIntake.targetPosition = intakeMotorMin
        if (motorIntake.currentPosition < intakeMotorMin+100 && servoIntake.position == servoIntakeUpPos){
            intake1.power = -intakePower
        }
    }

    fun lifting() {
        motorUp.targetPosition = liftingMotorMax
        motorUp.mode = DcMotor.RunMode.RUN_TO_POSITION
        opmodeTimer!!.resetTimer()
        if (opmodeTimer.elapsedTime >1000) {
            servoSlide.position = servoSlideUpPos
            servoDumper.position = servoIntakeDownPos
        }
    }

    fun dumping() {
        servoDumper.position = servoDumperDownPos
        opmodeTimer!!.resetTimer()
        if (opmodeTimer.elapsedTime >1000) {
        servoDumper.position = servoDumperUpPos
        servoSlide.position = servoIntakeDownPos
            }
    }

    fun returning() {
        motorUp.targetPosition = liftingMotorMin
        motorUp.mode = DcMotor.RunMode.RUN_TO_POSITION
    }

    fun cancellation() {
        motorIntake.targetPosition = intakeMotorMax
        opmodeTimer!!.resetTimer()
        if (opmodeTimer.elapsedTime >500) {
            intake1.power = -intakePower
        }
    }

    fun specimenPick() {
        servospecimen1.position = specimenservo1OpenPos
        servospecimen2.position = specimenservo2OpenPos
        motorSpecimen.targetPosition = specimenMotorMin
        if (motorSpecimen.currentPosition == specimenMotorMin +50) {
            opmodeTimer!!.resetTimer()
            if (opmodeTimer.elapsedTime >1000) {
                servospecimen1.position = specimenservo1ClosedPos
                servospecimen2.position = specimenservo2ClosedPos
            }
        }
    }

    fun specimenPut() {
        motorSpecimen.targetPosition = specimenMotorPlace
        servospecimen1.position = specimenservo1OpenPos
        servospecimen2.position = specimenservo2OpenPos
    }

    fun specimenLift() {
        motorSpecimen.targetPosition = specimenMotorMax

    }

    fun hang() {}
}