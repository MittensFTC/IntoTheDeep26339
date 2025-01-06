package pedroPathing.Mittens4point0

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import kotlin.math.abs
import kotlin.math.max


class TeleOperatonal : LinearOpMode() {
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

    var servoSlideDownPos: Double =
        0.0 //replace with empirical value for dumper starting pos on slide
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



        if (isStopRequested) return
        while (opModeIsActive()) {
            waitForStart()


            val y = -gamepad1.left_stick_y.toDouble() // Remember, Y stick value is reversed
            val x = gamepad1.left_stick_x.toDouble() // Counteract imperfect strafing
            val rx = gamepad1.right_stick_x.toDouble()
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
            val denominator = max(abs(y) + abs(x) + abs(rx), 1.0)
            val frontLeftPower = (y + x + rx) * speed / denominator
            val backLeftPower = (y - x + rx) * speed / denominator
            val frontRightPower = (y - x - rx) * speed / denominator
            val backRightPower = (y + x - rx) * speed / denominator
            motorFrontLeft.power = frontLeftPower
            motorBackLeft.power = backLeftPower
            motorFrontRight.power = frontRightPower
            motorBackRight.power = backRightPower
            when (step) {
                0 -> {
                    if (gamepad2.a){step = 1}
                }
                1 -> {
                    intaking()
                    if (gamepad2.b) { step = 2 }
                    else if (gamepad2.x) { step = 6 }
                }

                2 -> {
                    transferring()
                    sleep(1000)
                    step = 3
                }
                3 -> {
                    lifting()
                    if (gamepad2.y){ step = 4}
                }
                4 -> {
                    dumping()
                    sleep(500)
                    step = 5
                }

                5 -> {
                    returning()
                    step = 0
                }

                6 -> {
                    cancellation()
                    sleep(1000)
                    step = 0
                }
            }
        }
    }

    fun intaking() {
        motorIntake.targetPosition = intakeMotorMax
        motorIntake.mode = DcMotor.RunMode.RUN_TO_POSITION
        servoIntake.position = servoIntakeDownPos
        intake1.power = intakePower
    }

    fun transferring() {
        motorIntake.targetPosition = intakeMotorMin
        motorIntake.mode = DcMotor.RunMode.RUN_TO_POSITION
        servoIntake.position = servoIntakeUpPos
        if (motorIntake.targetPosition < intakeMotorMin + 50 && servoIntake.position == servoIntakeUpPos +- 0.1) {
            intake1.power = -intakePower
        }
    }

    fun lifting() {
        motorUp.targetPosition = liftingMotorMax
        motorUp.mode = DcMotor.RunMode.RUN_TO_POSITION
        sleep(500)
        servoSlide.position = servoSlideUpPos
        servoDumper.position = servoIntakeDownPos
    }

    fun dumping() {
        servoDumper.position = servoDumperDownPos
        sleep(1000)
        servoDumper.position = servoDumperUpPos
        servoSlide.position = servoIntakeDownPos
    }

    fun returning() {
        motorUp.targetPosition = liftingMotorMin
        motorUp.mode = DcMotor.RunMode.RUN_TO_POSITION
    }

    fun cancellation() {
        motorIntake.targetPosition = intakeMotorMax
        sleep(500)
        intake1.power = 1.0
    }

    fun specimenPick() {
        motorSpecimen.targetPosition = specimenMotorMin
        servospecimen1.position = specimenservo1ClosedPos
        servospecimen2.position = specimenservo2ClosedPos
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