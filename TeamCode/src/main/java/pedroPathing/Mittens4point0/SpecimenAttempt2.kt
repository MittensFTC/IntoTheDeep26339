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

@Autonomous(name = "SpecimenAuto", group = "Examples")
class SpecimenAttempt2 : LinearOpMode() {
    var robot: TeleOperatonal = TeleOperatonal()
    private val pathTimer: Timer? = null
    private val actionTimer: Timer? = null
    private val opmodeTimer: Timer? = null


    inner class Lift(hardwareMap: HardwareMap) {
        private val lift: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "ms")


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
                    lift.targetPosition = robot.specimenMotorMax
                    lift.mode = DcMotor.RunMode.RUN_TO_POSITION
                    lift.power = 0.8
                    initialized = true
                }


                val pos: Double = lift.currentPosition.toDouble()
                p.put("liftPos", pos)
                if (pos < robot.specimenMotorMax +- 200) {
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
                    lift.targetPosition = robot.specimenMotorMax
                    lift.mode = DcMotor.RunMode.RUN_TO_POSITION
                    lift.power = 0.8
                    initialized = true
                }


                val pos: Double = lift.currentPosition.toDouble()
                p.put("liftPos", pos)
                if (pos < robot.specimenMotorMin +- 200) {
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


        inner class LiftPlace : Action{
            private var initialized: Boolean = false

            override fun run(p: TelemetryPacket): Boolean {
                if (!initialized) {
                    lift.targetPosition = robot.specimenMotorPlace
                    lift.mode = DcMotor.RunMode.RUN_TO_POSITION
                    lift.power = 0.8
                    initialized = true
                }
                val pos: Double = lift.currentPosition.toDouble()
                p.put("liftPos", pos)
                if (pos < robot.specimenMotorPlace + -200) {
                    return true
                } else {
                    lift.power = 0.3
                    return false
                }
            }
        }
        fun liftPlace(): Action{
            return LiftPlace()
        }

    }



    inner class ServoSpecimen1(hardwareMap: HardwareMap) {
        private val servoSpecimen1: Servo = hardwareMap.get(Servo::class.java, "ss1")

        inner class ServoSpecimen1Open : Action {
            override fun run(p: TelemetryPacket): Boolean {
                servoSpecimen1.position = robot.specimenservo1OpenPos
                return false
            }
        }

        fun servoSpecimen1Open(): Action {
            return ServoSpecimen1Open()
        }

        inner class ServoSpecimen1Close : Action {
            override fun run(p: TelemetryPacket): Boolean {
                servoSpecimen1.position = robot.specimenservo1ClosedPos
                return false
            }
        }

        fun servoSpecimen1Close(): Action {
            return ServoSpecimen1Close()
        }
    }

    inner class ServoSpecimen2(hardwareMap: HardwareMap) {
        private val servoSpecimen2: Servo = hardwareMap.get(Servo::class.java, "ss2")

        inner class ServoSpecimen2Open : Action {
            override fun run(p: TelemetryPacket): Boolean {
                servoSpecimen2.position = robot.specimenservo2OpenPos
                return false
            }
        }

        fun servoSpecimen2Open(): Action {
            return ServoSpecimen2Open()
        }

        inner class ServoSpecimen2Close : Action {
            override fun run(p: TelemetryPacket): Boolean {
                servoSpecimen2.position = robot.specimenservo2ClosedPos
                return false
            }
        }

        fun servoSpecimen2Close(): Action {
            return ServoSpecimen2Close()
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


            override fun run(p: TelemetryPacket): Boolean {
                p.put("INTAKE", "ON!!!!")
                intake.power = -1.0

                return true
            }
        }

        fun intakeOut(): Action {
            return IntakeOut()
        }


        inner class IntakeIn : Action {

            override fun run(p: TelemetryPacket): Boolean {
                p.put("INTAKE", "ON!!!!")
                intake.power = 1.0

                return true
            }
        }

        fun intakeIn(): Action {
            return IntakeIn()
        }
    }


    private var follower: Follower? = null

    private var pathState = 0




    private var telemetryA: Telemetry? = null
    var lift: Lift = Lift(hardwareMap)
    var servoSpecimen1: ServoSpecimen1 = ServoSpecimen1(hardwareMap)
    var servoSpecimen2: ServoSpecimen2 = ServoSpecimen2(hardwareMap)
    var actuator: Actuator = Actuator(hardwareMap)
    var servoIntake: Intake = Intake(hardwareMap)

    fun intaking(){
        actuator.intakeOut()
        servoIntake.intakeIn()
    }
    fun outtaking(){
        actuator.intakeOut()
        servoIntake.intakeOut()
    }
    fun ActuatorIn(){
        actuator.intakeIn()
    }

    fun specimenPick() {
        lift.liftDown()
        servoSpecimen1.servoSpecimen1Open()
        servoSpecimen2.servoSpecimen2Open()
    }
    fun clawClose(){
        servoSpecimen1.servoSpecimen1Close()
        servoSpecimen2.servoSpecimen2Close()
    }

    fun specimenPut() {
        lift.liftPlace()
        servoSpecimen1.servoSpecimen1Open()
        servoSpecimen2.servoSpecimen2Open()
    }

    fun specimenLift() {
        lift.liftUp()
    }

    fun returning() {
        runBlocking(lift.liftDown())
        SleepAction(0.5)
    }
    private val startPose = Pose(8.0, 56.0, Math.toRadians(0.0))
    private var specimen1Pose = Pose(39.0, 84.0, Math.toRadians(270.0))
    private var sample1Pose = Pose(30.0, 35.0, Math.toRadians(315.0))
    private var sample1Outtake = Pose(30.0, 35.0, Math.toRadians(220.0))
    private var sample2Pose = Pose(30.0, 26.0, Math.toRadians(315.0))
    private var sample2Outtake = Pose(30.0, 26.0, Math.toRadians(220.0))
    private var sample3Pose = Pose(30.0, 17.0, Math.toRadians(315.0))
    private var sample3Outtake = Pose (28.0, 26.0, Math.toRadians(180.0))
    private var pickUpGetSet = Pose (28.0, 26.0, Math.toRadians(90.0))
    private var pickUp = Pose (11.0, 26.0, Math.toRadians(90.0))
    private var specimen2Pose = Pose(39.0, 82.0, Math.toRadians(270.0))
    private var specimen3Pose = Pose(39.0, 80.0, Math.toRadians(270.0))
    private var specimen4Pose = Pose(39.0, 78.0, Math.toRadians(270.0))
    private var specimen5Pose = Pose(39.0, 76.0, Math.toRadians(270.0))
    private var park = Pose(20.0, 39.0, Math.toRadians(225.0))


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
/*        chamber = follower!!.pathBuilder()
            .addPath(BezierLine(Point(startPose), Point(specimen1)))
            .setLinearHeadingInterpolation(startPose.heading, specimen1.heading)
            .build()*/

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

