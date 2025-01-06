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
import com.pedropathing.pathgen.Path
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
                    lift.power = 1.0
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
                    lift.power = 1.0
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
                    lift.power = 1.0
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
        runBlocking(ParallelAction(actuator.intakeOut(), servoIntake.intakeIn()))

    }
    fun outtaking(){
        runBlocking(ParallelAction(actuator.intakeOut(), servoIntake.intakeOut()))
    }
    fun actuatorIn(){
       runBlocking(actuator.intakeIn())
    }
    fun actuatorOut(){
        runBlocking(actuator.intakeOut())
    }


    fun specimenPick() {
        runBlocking(ParallelAction(lift.liftDown(), servoSpecimen1.servoSpecimen1Open(),servoSpecimen2.servoSpecimen2Open()))

    }
    fun clawClose(){
        runBlocking(ParallelAction(servoSpecimen1.servoSpecimen1Close(), servoSpecimen2.servoSpecimen2Close()))

    }

    fun specimenPut() {
        runBlocking(lift.liftPlace())
        runBlocking(SleepAction(0.3))
        clawOpen()
    }

    fun specimenLift() {
        clawClose()
        runBlocking(SleepAction(0.2))
        runBlocking(lift.liftUp())
    }

    fun returning() {
        runBlocking(lift.liftDown())
        clawClose()
    }
    fun clawOpen(){
        runBlocking(ParallelAction(servoSpecimen1.servoSpecimen1Open(), servoSpecimen2.servoSpecimen2Open()))
    }
    private val startPose = Pose(8.0, 56.0, Math.toRadians(0.0))
    private var specimen1Pose = Pose(39.0, 84.0, Math.toRadians(180.0))
    private var sample1Pose = Pose(30.0, 35.0, Math.toRadians(315.0))
    private var sample1Outtake = Pose(30.0, 35.0, Math.toRadians(220.0))
    private var sample2Pose = Pose(30.0, 26.0, Math.toRadians(315.0))
    private var sample2Outtake = Pose(30.0, 26.0, Math.toRadians(220.0))
    private var sample3Pose = Pose(30.0, 17.0, Math.toRadians(315.0))
    private var sample3Outtake = Pose (28.0, 26.0, Math.toRadians(180.0))
    private var pickUpGetSet = Pose (28.0, 26.0, Math.toRadians(0.0))
    private var pickUp = Pose (11.0, 26.0, Math.toRadians(0.0))
    private var specimen2Pose = Pose(39.0, 82.0, Math.toRadians(180.0))
    private var specimen3Pose = Pose(39.0, 80.0, Math.toRadians(180.0))
    private var specimen4Pose = Pose(39.0, 78.0, Math.toRadians(180.0))
    private var specimen5Pose = Pose(39.0, 76.0, Math.toRadians(180.0))
    private var park = Pose(20.0, 39.0, Math.toRadians(225.0))


    private var specimen1: PathChain? = null
    private var sample1PickUp: PathChain? = null
    private var sample1DropOff: PathChain? = null
    private var sample2PickUp: PathChain? = null
    private var sample2DropOff: PathChain? = null
    private var sample3PickUp: PathChain? = null
    private var sample3DropOff: PathChain? = null
    private var pickUpGetReady: PathChain? = null
    private var pickUpSpecimen2: PathChain? = null
    private var specimen2: PathChain? = null
    private var pickUpSpecimen3: PathChain? = null
    private var specimen3: PathChain? = null
    private var pickUpSpecimen4: PathChain? = null
    private var specimen4: PathChain? = null
    private var pickUpSpecimen5: PathChain? = null
    private var specimen5: PathChain? = null
    private var parked: PathChain? = null

    fun buildPath() {
        specimen1 = follower!!.pathBuilder()
            .addPath(BezierLine(Point(startPose), Point(specimen1Pose)))
            .setLinearHeadingInterpolation(startPose.heading, specimen1Pose.heading)
            .build()
        sample1PickUp = follower!!.pathBuilder()
            .addPath(BezierLine(Point(specimen1Pose), Point(sample1Pose)))
            .setLinearHeadingInterpolation(specimen1Pose.heading, sample1Pose.heading)
            .build()
        sample1DropOff = follower!!.pathBuilder()
            .addPath(BezierLine(Point(sample1Pose), Point(sample1Outtake)))
            .setLinearHeadingInterpolation(sample1Pose.heading, sample1Outtake.heading)
            .build()
        sample2PickUp = follower!!.pathBuilder()
            .addPath(BezierLine(Point(sample1Outtake), Point(sample2Pose)))
            .setLinearHeadingInterpolation(sample1Outtake.heading, sample2Pose.heading)
            .build()
        sample2DropOff = follower!!.pathBuilder()
            .addPath(BezierLine(Point(sample2Pose), Point(sample2Outtake)))
            .setLinearHeadingInterpolation(sample2Pose.heading, sample2Outtake.heading)
            .build()
        sample3PickUp = follower!!.pathBuilder()
            .addPath(BezierLine(Point(sample2Outtake), Point(sample3Pose)))
            .setLinearHeadingInterpolation(sample2Outtake.heading, sample3Pose.heading)
            .build()
        sample3DropOff = follower!!.pathBuilder()
            .addPath(BezierLine(Point(sample3Pose), Point(sample3Outtake)))
            .setLinearHeadingInterpolation(sample3Pose.heading, sample3Outtake.heading)
            .build()
        pickUpGetReady = follower!!.pathBuilder()
            .addPath(BezierLine(Point(sample3Outtake), Point(pickUpGetSet)))
            .setLinearHeadingInterpolation(sample3Outtake.heading, pickUpGetSet.heading)
            .build()
        pickUpSpecimen2 = follower!!.pathBuilder()
            .addPath(BezierLine(Point(pickUpGetSet), Point(pickUp)))
            .setLinearHeadingInterpolation(pickUpGetSet.heading, pickUp.heading)
            .build()
        specimen2 = follower!!.pathBuilder()
            .addPath(BezierLine(Point(pickUp), Point(specimen2Pose)))
            .setLinearHeadingInterpolation(pickUp.heading, specimen2Pose.heading)
            .build()
        pickUpSpecimen3 = follower!!.pathBuilder()
            .addPath(BezierLine(Point(specimen2Pose), Point(pickUp)))
            .setLinearHeadingInterpolation(specimen2Pose.heading, pickUp.heading)
            .build()
        specimen3 = follower!!.pathBuilder()
            .addPath(BezierLine(Point(pickUp), Point(specimen3Pose)))
            .setLinearHeadingInterpolation(pickUp.heading, specimen3Pose.heading)
            .build()
        pickUpSpecimen4 = follower!!.pathBuilder()
            .addPath(BezierLine(Point(specimen3Pose), Point(pickUp)))
            .setLinearHeadingInterpolation(specimen3Pose.heading, pickUp.heading)
            .build()
        specimen4 = follower!!.pathBuilder()
            .addPath(BezierLine(Point(pickUp), Point(specimen4Pose)))
            .setLinearHeadingInterpolation(pickUp.heading, specimen4Pose.heading)
            .build()
        pickUpSpecimen5 = follower!!.pathBuilder()
            .addPath(BezierLine(Point(specimen4Pose), Point(pickUp)))
            .setLinearHeadingInterpolation(specimen4Pose.heading, pickUp.heading)
            .build()
        specimen5 = follower!!.pathBuilder()
            .addPath(BezierLine(Point(pickUp), Point(specimen5Pose)))
            .setLinearHeadingInterpolation(pickUp.heading, specimen5Pose.heading)
            .build()
        parked = follower!!.pathBuilder()
            .addPath(BezierLine(Point(specimen5Pose), Point(park)))
            .setLinearHeadingInterpolation(specimen5Pose.heading, park.heading)
            .build()
    }

    fun setPathState(pState: Int) {
        pathState = pState
        pathTimer!!.resetTimer()
    }

    fun autonomousPathUpdate() {
        when (pathState) {
            0 -> {
                specimenLift()
                clawClose()
                follower!!.followPath(specimen1)
                if (!follower!!.isBusy){
                    follower!!.followPath(specimen1, true)
                    specimenPut()
                    runBlocking(SleepAction(0.2))
                    returning()
                    intaking()
                    setPathState(1)
                }
            }

            1 -> {
            follower!!.followPath(sample1PickUp)
                if (!follower!!.isBusy){
                    follower!!.followPath(sample1PickUp, true)
                    runBlocking(SleepAction(0.2))
                    setPathState(2)
                }
            }

            2 -> {
                follower!!.followPath(sample1DropOff)
                if (!follower!!.isBusy){
                    follower!!.followPath(sample1DropOff, true)
                    outtaking()
                    runBlocking(SleepAction(0.2))
                    setPathState(3)
                }

            }

            3 -> {
                follower!!.followPath(sample2PickUp)
                intaking()
                if (!follower!!.isBusy){
                    follower!!.followPath(sample2PickUp, true)
                    runBlocking(SleepAction(0.2))
                    setPathState(4)
                }
            }

            4 -> {
                follower!!.followPath(sample2DropOff)
                if (!follower!!.isBusy){
                    follower!!.followPath(sample2DropOff, true)
                    outtaking()
                    runBlocking(SleepAction(0.2))
                    setPathState(5)
                }

            }

            5 -> {
                follower!!.followPath(sample3PickUp)
                intaking()
                if (!follower!!.isBusy){
                    follower!!.followPath(sample3PickUp, true)
                    runBlocking(SleepAction(0.2))
                    setPathState(6)
                }


            }

            6 -> {
                follower!!.followPath(sample3DropOff)
                if(!follower!!.isBusy){
                    follower!!.followPath(sample3DropOff)
                    outtaking()
                    runBlocking(SleepAction(0.2))
                    setPathState(7)
                }
            }
            7 ->{
                follower!!.followPath(pickUpGetReady)
                specimenPick()
                if (!follower!!.isBusy){
                    setPathState(8)
                }
            }
            8 ->{
                follower!!.followPath(pickUpSpecimen2)
                clawOpen()
                if(!follower!!.isBusy){
                    follower!!.followPath(pickUpSpecimen2, true)
                    runBlocking(SleepAction(0.2))
                    specimenLift()
                    setPathState(9)
                }
            }
            9 ->{
                follower!!.followPath(specimen2)
                if(!follower!!.isBusy){
                    follower!!.followPath(specimen2, true)
                    specimenPut()
                    runBlocking(SleepAction(0.2))
                    specimenPick()
                    setPathState(10)
                }
            }
            10 ->{
                follower!!.followPath(pickUpSpecimen3)
                if(!follower!!.isBusy){
                    follower!!.followPath(pickUpSpecimen3, true)
                    runBlocking(SleepAction(0.2))
                    specimenLift()
                    setPathState(11)
                }
            }
            11 ->{
                follower!!.followPath(specimen3)
                if(!follower!!.isBusy){
                    follower!!.followPath(specimen3, true)
                    specimenPut()
                    runBlocking(SleepAction(0.2))
                    specimenPick()
                    setPathState(12)
                }
            }
            12 -> {
                follower!!.followPath(pickUpSpecimen4)
                if(!follower!!.isBusy){
                    follower!!.followPath(pickUpSpecimen4, true)
                    runBlocking(SleepAction(0.2))
                    specimenLift()
                    setPathState(13)
                }
            }
            13 -> {
                follower!!.followPath(specimen4)
                if(!follower!!.isBusy){
                    follower!!.followPath(specimen4, true)
                    specimenPut()
                    runBlocking(SleepAction(0.2))
                    specimenPick()
                    setPathState(14)
                }
            }
            14 -> {
                follower!!.followPath(pickUpSpecimen5)
                if(!follower!!.isBusy){
                    follower!!.followPath(pickUpSpecimen5, true)
                    runBlocking(SleepAction(0.2))
                    specimenLift()
                    setPathState(15)
                }
            }
            15 -> {
                if(!follower!!.isBusy){
                    follower!!.followPath(specimen5, true)
                    specimenPut()
                    runBlocking(SleepAction(0.2))
                    specimenPick()
                    setPathState(14)
                }
            }
            16 -> {
                follower!!.followPath(parked)
                actuatorOut()

            }

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
        opmodeTimer!!.resetTimer()
        follower!!.update()
        autonomousPathUpdate()
        while (opModeIsActive() && !isStopRequested) {
            if (opmodeTimer.elapsedTime > 28.0) {
                setPathState(16)
            }
        }


    }
}

