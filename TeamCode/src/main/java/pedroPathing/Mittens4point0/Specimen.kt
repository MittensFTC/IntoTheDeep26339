package pedroPathing.Mittens4point0

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.pedropathing.follower.Follower
import com.pedropathing.localization.Pose
import com.pedropathing.pathgen.BezierCurve
import com.pedropathing.pathgen.BezierLine
import com.pedropathing.pathgen.PathChain
import com.pedropathing.pathgen.Point
import com.pedropathing.util.Constants
import com.pedropathing.util.Timer
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.Telemetry
import pedroPathing.constants.FConstants
import pedroPathing.constants.LConstants

@Autonomous(name = "Samples", group = "Examples")
class Specimen : LinearOpMode() {
    var robot: TeleOperatonal = TeleOperatonal()
    private val pathTimer: Timer? = null
    private val actionTimer: Timer? = null
    private val opmodeTimer: Timer? = null


    inner class Lift(hardwareMap: HardwareMap) {
        private val lift: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "mu")


        init {
            lift.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            lift.direction = DcMotorSimple.Direction.FORWARD
            lift.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            lift.mode = DcMotor.RunMode.RUN_USING_ENCODER
        }


        inner class LiftUp : Action {
            private var initialized: Boolean = false


            override fun run(p: TelemetryPacket): Boolean {
                if (!initialized) {
                    lift.targetPosition = robot.liftingMotorMax
                    lift.mode = DcMotor.RunMode.RUN_TO_POSITION
                    lift.power = 0.8
                    initialized = true
                }


                val pos: Double = lift.currentPosition.toDouble()
                p.put("liftPos", pos)
                if (pos < robot.liftingMotorMax - 200) {
                    return true
                } else {
                    lift.power = 0.8
                    return false
                }
            }
        }

        fun liftUp(): Action {
            return LiftUp()
        }

        inner class LiftDown : Action {
            private var initialized: Boolean = false

            override fun run(p: TelemetryPacket): Boolean {
                if (!initialized) {
                    lift.targetPosition = robot.liftingMotorMin
                    lift.mode = DcMotor.RunMode.RUN_TO_POSITION
                    lift.power = -0.8
                    initialized = true
                }


                val pos: Double = lift.currentPosition.toDouble()
                p.put("liftPos", pos)
                if (pos < 200) {
                    return true
                } else {
                    lift.power = 0.3
                    return false
                }
            }
        }

        fun liftDown(): Action {
            return LiftDown()
        }
    }

    inner class Actuator(hardwareMap: HardwareMap) {
        private val actuator: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "mi")

        init {
            actuator.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            actuator.direction = DcMotorSimple.Direction.FORWARD
            actuator.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            actuator.mode = DcMotor.RunMode.RUN_USING_ENCODER
        }

        inner class IntakeOut : Action {
            private var initialized: Boolean = false

            override fun run(p: TelemetryPacket): Boolean {
                if (!initialized) {
                    actuator.targetPosition = robot.intakeMotorMax
                    actuator.mode = DcMotor.RunMode.RUN_TO_POSITION
                    actuator.power = 0.8
                    initialized = true
                }


                val pos: Double = actuator.currentPosition.toDouble()
                p.put("liftPos", pos)
                if (pos < robot.intakeMotorMax - 200) {
                    return true
                } else {
                    actuator.power = 0.3
                    return false
                }
            }
        }

        fun intakeOut(): Action {
            return IntakeOut()
        }

        inner class IntakeIn : Action {
            private var initialized: Boolean = false

            override fun run(p: TelemetryPacket): Boolean {
                if (!initialized) {
                    actuator.targetPosition = robot.intakeMotorMin
                    actuator.mode = DcMotor.RunMode.RUN_TO_POSITION
                    actuator.power = -0.8
                    initialized = true
                }


                val pos: Double = actuator.currentPosition.toDouble()
                p.put("liftPos", pos)
                if (pos < robot.intakeMotorMin + 200) {
                    return true
                } else {
                    actuator.power = 0.3
                    return false
                }
            }
        }

        fun intakeIn(): Action {
            return IntakeIn()
        }
    }

    inner class Intake(hardwareMap: HardwareMap) {
        private val intake: CRServo = hardwareMap.crservo["si1"]


        inner class IntakeOut : Action {
            var cycles: Int = 0


            override fun run(p: TelemetryPacket): Boolean {
                p.put("INTAKE", "ON!!!!")
                intake.power = 1.0


                sleep(1000)
                p.put("INTAKE", "Off")
                cycles++
                return cycles < 2
            }
        }

        fun intakeOut(): Action {
            return IntakeOut()
        }


        inner class IntakeIn : Action {
            var cycles: Int = 0

            override fun run(p: TelemetryPacket): Boolean {
                p.put("INTAKE", "ON!!!!")
                intake.power = -1.0


                sleep(1000)
                p.put("INTAKE", "Off")
                cycles++
                return cycles < 2
            }
        }

        fun intakeIn(): Action {
            return IntakeIn()
        }
    }

    inner class IntakeServo(hardwareMap: HardwareMap) {
        private val intakeServo: Servo = hardwareMap.get(Servo::class.java, "si1")

        inner class IntakeDown : Action {
            override fun run(p: TelemetryPacket): Boolean {
                intakeServo.position = robot.servoIntakeDownPos
                return false
            }
        }

        fun intakeDown(): Action {
            return IntakeDown()
        }

        inner class IntakeUp : Action {
            override fun run(p: TelemetryPacket): Boolean {
                intakeServo.position = robot.servoIntakeUpPos
                return false
            }
        }

        fun intakeUp(): Action {
            return IntakeUp()
        }
    }

    inner class ServoSlide(hardwareMap: HardwareMap) {
        private val servoSlide: Servo = hardwareMap.get(Servo::class.java, "ss")

        inner class ServoSlideDown : Action {
            override fun run(p: TelemetryPacket): Boolean {
                servoSlide.position = robot.servoSlideDownPos
                return false
            }
        }

        fun servoSlideDown(): Action {
            return ServoSlideDown()
        }

        inner class ServoSlideUp : Action {
            override fun run(p: TelemetryPacket): Boolean {
                servoSlide.position = robot.servoSlideUpPos
                return false
            }
        }

        fun servoSlideUp(): Action {
            return ServoSlideUp()
        }
    }

    inner class ServoDumper(hardwareMap: HardwareMap) {
        private val servoDumper: Servo = hardwareMap.get(Servo::class.java, "sd")

        inner class ServoDumpUp : Action {
            override fun run(p: TelemetryPacket): Boolean {
                servoDumper.position = robot.servoDumperUpPos
                return false
            }
        }

        fun servoDumpUp(): Action {
            return ServoDumpUp()
        }

        inner class ServoDumpDown : Action {
            override fun run(p: TelemetryPacket): Boolean {
                servoDumper.position = robot.servoDumperDownPos
                return false
            }
        }

        fun servoDumpDown(): Action {
            return ServoDumpDown()
        }
    }


    private var follower: Follower? = null

    private var pathState = 0


    private val startPose = Pose(8.0, 56.0, Math.toRadians(0.0))
    private val specimen1 = Pose(36.0,80.0, Math.toRadians(-90.0))
    private val push1 = Pose(60.0, 15.0, Math.toRadians(0.0))
    private val push1C1 = Pose(10.0,36.0)
    private val push1C2 = Pose(40.0,35.0)
    private val push2 = Pose(60.0,15.0,Math.toRadians(0.0))
    private val push2C1 = Pose(89.0, 16.0)
    private val push2C2 = Pose(-90.0, 30.0)
    private val push2C3 = Pose(40.0,28.0)
    private val push2C4 = Pose(95.0, 31.0)
    private val push3 = Pose(66.0,9.0,Math.toRadians(0.0))
    private val push3C1 = Pose(-30.0, 21.0)
    private val push3C2 = Pose(36.0, 30.0)
    private val push4 = Pose(20.0, 9.0, Math.toRadians(0.0))
    private val getReadyForPickUp = Pose(26.0, 27.0, Math.toRadians(90.0))
    private val pickUp = Pose(12.0,26.0,Math.toRadians(90.0))
    private val specimen2 = Pose(36.0,76.0, Math.toRadians(-90.0))
    private val specimen3 = Pose(36.0, 72.0, Math.toRadians(-90.0))
    private val specimen4 = Pose(36.0, 68.0,Math.toRadians(-90.0))
    private val specimen5 = Pose(36.0, 64.0, Math.toRadians(-90.0))
    private val park = Pose(19.0, 31.0, Math.toRadians(90.0))




    private var telemetryA: Telemetry? = null
    var lift: Lift = Lift(hardwareMap)
    var actuator: Actuator = Actuator(hardwareMap)
    var crServoIntake: Intake = Intake(hardwareMap)
    var servoSlide: ServoSlide = ServoSlide(hardwareMap)
    var servoDumper: ServoDumper = ServoDumper(hardwareMap)
    var servoIntake: IntakeServo = IntakeServo(hardwareMap)

    fun intaking() {
        runBlocking(
            ParallelAction(
                actuator.intakeOut(),
                servoIntake.intakeDown(),
                crServoIntake.intakeIn()
            )
        )
    }

    fun transferring() {
        runBlocking(
            SequentialAction(
                actuator.intakeIn(),
                servoIntake.intakeUp(),
                SleepAction(1.0),
                crServoIntake.intakeOut()
            )
        )
    }

    fun lifting() {
        runBlocking(
            SequentialAction(
                lift.liftUp(),
                SleepAction(1.0),
                servoSlide.servoSlideUp(),
                servoDumper.servoDumpUp()
            )
        )
    }

    fun dumping() {
        runBlocking(
            SequentialAction(
                servoDumper.servoDumpDown(),
                SleepAction(1.0),
                servoDumper.servoDumpUp(),
                servoSlide.servoSlideDown()
            )
        )
    }

    fun returning() {
        runBlocking(lift.liftDown())
        SleepAction(0.5)
    }
    private var chamber: PathChain? = null
    private var sample1GetSet: PathChain? = null
    private var sample2GetSet: PathChain? = null
    private var sameple3GetSet: PathChain? = null
    private var sample3In: PathChain? = null
    private var specimenGetSet: PathChain? = null
    private var specimen1PickUp: PathChain? = null
    private var chamber1: PathChain? = null
    private var specimen2PickUp: PathChain? = null
    private var chamber2: PathChain? = null
    private var specimen3PickUp: PathChain? = null
    private var chamber3: PathChain? = null
    private var specimen4PickUp: PathChain? = null
    private var chamber4: PathChain? = null
    private var parked: PathChain? = null

    fun buildPath() {
        chamber = follower!!.pathBuilder()
            .addPath(BezierLine(Point(startPose), Point(specimen1)))
            .setLinearHeadingInterpolation(startPose.heading, specimen1.heading)
            .build()
        sample1GetSet = follower!!.pathBuilder()
            .addPath(BezierCurve(Point(specimen1), Point(push1C1), Point(push1C2), Point(push1)))
            .setLinearHeadingInterpolation(specimen1.heading, push1.heading)
            .build()
        sample2GetSet = follower!!.pathBuilder()
            .addPath(BezierCurve(Point(push1),Point(push2C1), Point(push2C2), Point(push2C3), Point(push2C4), Point(push2)))
            .setLinearHeadingInterpolation(push1.heading, push2.heading)
            .build()
        sameple3GetSet = follower!!.pathBuilder()
            .addPath(BezierCurve(Point(push2), Point(push3C1), Point(push3C2), Point(push3)))
            .setLinearHeadingInterpolation(push2.heading,push3.heading)
            .build()
        sample3In = follower!!.pathBuilder()
            .addPath(BezierLine(Point(push3), Point(push4)))
            .setLinearHeadingInterpolation(push3.heading,push4.heading)
            .build()
        specimenGetSet = follower!!.pathBuilder()
            .addPath(BezierLine(Point(push4), Point(getReadyForPickUp)))
            .setLinearHeadingInterpolation(push4.heading, getReadyForPickUp.heading)
            .build()
        specimen1PickUp = follower!!.pathBuilder()
            .addPath(BezierLine(Point(getReadyForPickUp), Point(pickUp)))
            .setLinearHeadingInterpolation(getReadyForPickUp.heading, pickUp.heading)
            .build()
        chamber1 = follower!!.pathBuilder()
            .addPath(BezierLine(Point(pickUp), Point(specimen2)))
            .setLinearHeadingInterpolation(pickUp.heading, specimen2.heading)
            .build()
        specimen2PickUp = follower!!.pathBuilder()
            .addPath(BezierLine(Point(specimen2), Point(pickUp)))
            .setLinearHeadingInterpolation(specimen2.heading, pickUp.heading)
            .build()
        chamber2 = follower!!.pathBuilder()
            .addPath(BezierLine(Point(pickUp), Point(specimen3)))
            .setLinearHeadingInterpolation(pickUp.heading, specimen3.heading)
            .build()
        specimen3PickUp = follower!!.pathBuilder()
            .addPath(BezierLine(Point(specimen3), Point(pickUp)))
            .setLinearHeadingInterpolation(specimen3.heading, pickUp.heading)
            .build()
        chamber3 = follower!!.pathBuilder()
            .addPath(BezierLine(Point(pickUp), Point(specimen4)))
            .setLinearHeadingInterpolation(pickUp.heading, specimen4.heading)
            .build()
        specimen4PickUp = follower!!.pathBuilder()
            .addPath(BezierLine(Point(specimen4), Point(pickUp)))
            .setLinearHeadingInterpolation(specimen4.heading, pickUp.heading)
            .build()
        chamber4 = follower!!.pathBuilder()
            .addPath(BezierLine(Point(pickUp),Point(specimen5)))
            .setLinearHeadingInterpolation(pickUp.heading, specimen5.heading)
            .build()
        parked = follower!!.pathBuilder()
            .addPath(BezierLine(Point(specimen5), Point(park)))
            .setLinearHeadingInterpolation(specimen5.heading, park.heading)
            .build()


    }

    fun setPathState(pState: Int) {
        pathState = pState
        pathTimer!!.resetTimer()
    }

    fun autonomousPathUpdate() {
        when (pathState) {
            0 -> {


            }

            1 -> {


            }

            2 -> {


            }

            3 -> {


            }

            4 -> {


            }

            5 -> {


            }

            6 -> {}
        }
    }


    override fun runOpMode() {
        follower = Follower(hardwareMap)


        follower!!.setStartingPose(startPose)
        buildPath()


        telemetryA = MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().telemetry)
        (telemetryA as MultipleTelemetry).addLine("hi people" + "It's me your Bestie!!!" + "I will not play omori")
        (telemetryA as MultipleTelemetry).update()


        while (!isStopRequested && !opModeIsActive()) {
            //hehe
        }


        waitForStart()
        if (isStopRequested) return
        follower!!.update()
        autonomousPathUpdate()




    }
}

