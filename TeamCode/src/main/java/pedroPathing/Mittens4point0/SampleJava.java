package pedroPathing.Mittens4point0;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@Autonomous(name = "Samples", group = "Examples")
public class SampleJava extends LinearOpMode {
    TeleOperatonalJava robot = new TeleOperatonalJava();
    private Timer pathTimer, actionTimer, opmodeTimer;




    public class Lift {
        private final DcMotorEx lift;


        public Lift(HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotorEx.class, "mu");
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setDirection(DcMotorSimple.Direction.FORWARD);
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }


        public class LiftUp implements Action {
            private boolean initialized = false;


            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setTargetPosition(robot.liftingMotorMax);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(0.8);
                    initialized = true;
                }


                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < robot.liftingMotorMax - 200) {
                    return true;
                } else {
                    lift.setPower(0.8);
                    return false;
                }
            }
        }

        public Action liftUp() {
            return new LiftUp();
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setTargetPosition(robot.liftingMotorMin);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(-0.8);
                    initialized = true;
                }


                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 200) {
                    return true;
                } else {
                    lift.setPower(0.3);
                    return false;
                }
            }
        }

        public Action liftDown() {
            return new LiftDown();
        }

    }

    public class Actuator {
        private final DcMotorEx actuator;

        public Actuator(HardwareMap hardwareMap) {
            actuator = hardwareMap.get(DcMotorEx.class, "mi");
            actuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            actuator.setDirection(DcMotorSimple.Direction.FORWARD);
            actuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            actuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public class IntakeOut implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    actuator.setTargetPosition(robot.intakeMotorMax);
                    actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    actuator.setPower(0.8);
                    initialized = true;
                }


                double pos = actuator.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < robot.intakeMotorMax - 200) {
                    return true;
                } else {
                    actuator.setPower(0.3);
                    return false;
                }
            }

        }

        public Action intakeOut() {
            return new IntakeOut();
        }

        public class IntakeIn implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    actuator.setTargetPosition(robot.intakeMotorMin);
                    actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    actuator.setPower(-0.8);
                    initialized = true;
                }


                double pos = actuator.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < robot.intakeMotorMin + 200) {
                    return true;
                } else {
                    actuator.setPower(0.3);
                    return false;
                }
            }

        }

        public Action intakeIn() {
            return new IntakeIn();
        }
    }

    public class Intake {
        private final CRServo intake;


        public Intake(HardwareMap hardwareMap) {
            intake = hardwareMap.crservo.get("si1");
        }


        public class IntakeOut implements Action {
            int cycles = 0;


            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                packet.put("INTAKE", "ON!!!!");
                intake.setPower(1);


                sleep(1000);
                packet.put("INTAKE", "Off");
                cycles++;
                return cycles < 2;
            }
        }

        public Action intakeOut() {
            return new IntakeOut();
        }


        public class IntakeIn implements Action {

            int cycles = 0;

            public boolean run(@NonNull TelemetryPacket packet) {
                packet.put("INTAKE", "ON!!!!");
                intake.setPower(-1);


                sleep(1000);
                packet.put("INTAKE", "Off");
                cycles++;
                return cycles < 2;
            }
        }

        public Action intakeIn() {
            return new IntakeIn();
        }
    }

    public class IntakeServo {
        private Servo intakeServo;

        public IntakeServo(HardwareMap hardwareMap) {
            intakeServo = hardwareMap.get(Servo.class, "si1");
        }

        public class IntakeDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeServo.setPosition(robot.servoIntakeDownPos);
                return false;
            }
        }

        public Action intakeDown() {
            return new IntakeDown();
        }

        public class IntakeUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeServo.setPosition(robot.servoIntakeUpPos);
                return false;
            }
        }

        public Action intakeUp() {
            return new IntakeUp();
        }
    }

    public class ServoSlide {
        private Servo servoSlide;

        public ServoSlide(HardwareMap hardwareMap) {
            servoSlide = hardwareMap.get(Servo.class, "ss");
        }

        public class ServoSlideDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                servoSlide.setPosition(robot.servoSlideDownPos);
                return false;
            }
        }

        public Action servoSlideDown() {
            return new ServoSlideDown();
        }

        public class ServoSlideUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                servoSlide.setPosition(robot.servoSlideUpPos);
                return false;
            }
        }

        public Action servoSlideUp() {
            return new ServoSlideUp();
        }
    }

    public class ServoDumper {
        private Servo servoDumper;

        public ServoDumper(HardwareMap hardwareMap) {
            servoDumper = hardwareMap.get(Servo.class, "sd");
        }

        public class ServoDumpUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                servoDumper.setPosition(robot.servoDumperUpPos);
                return false;
            }
        }

        public Action servoDumpUp() {
            return new ServoDumpUp();
        }

        public class ServoDumpDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                servoDumper.setPosition(robot.servoDumperDownPos);
                return false;
            }
        }

        public Action servoDumpDown() {
            return new ServoDumpDown();
        }
    }


    private Follower follower;

    private int pathState;

    private final Pose startPose = new Pose(8, 80, Math.toRadians(0));
    private final Pose bucket = new Pose(17, 128, Math.toRadians(-45));
    private final Pose sample1 = new Pose(28, 120, Math.toRadians(0));
    private final Pose sample1Curve = new Pose(19, 118);
    private final Pose sample2 = new Pose(28, 132, Math.toRadians(0));
    private final Pose sample2Curve = new Pose(19, 133);
    private final Pose sample3 = new Pose(45, 127, Math.toRadians(90));
    private final Pose interPose = new Pose(24, -24, Math.toRadians(90));
    private final Pose endPose = new Pose(24, 24, Math.toRadians(45));

    private PathChain triangle, bucket1, bucket2, bucket3, bucket4, sample11, sample22, sample33;


    private Telemetry telemetryA;
    Lift lift = new Lift(hardwareMap);
    Actuator actuator = new Actuator(hardwareMap);
    Intake crServoIntake = new Intake(hardwareMap);
    ServoSlide servoSlide = new ServoSlide(hardwareMap);
    ServoDumper servoDumper = new ServoDumper(hardwareMap);
    IntakeServo servoIntake = new IntakeServo(hardwareMap);

    public void intaking() {
        Actions.runBlocking(new ParallelAction(
                actuator.intakeOut(),
                servoIntake.intakeDown(),
                crServoIntake.intakeIn()));


    }

    public void transferring() {
        Actions.runBlocking(new SequentialAction(
                actuator.intakeIn(),
                servoIntake.intakeUp(),
                new SleepAction(1),
                crServoIntake.intakeOut()));


    }

    public void lifting() {
        Actions.runBlocking(new SequentialAction(
                lift.liftUp(),
                new SleepAction(1),
                servoSlide.servoSlideUp(),
                servoDumper.servoDumpUp()));

    }

    public void dumping() {
        Actions.runBlocking(new SequentialAction(
                servoDumper.servoDumpDown(),
                new SleepAction(1),
                servoDumper.servoDumpUp(),
                servoSlide.servoSlideDown()));

    }

    public void returning() {
        Actions.runBlocking(lift.liftDown());
        new SleepAction(0.5);
    }

    public void buildPath() {

        bucket1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(bucket)))
                .setLinearHeadingInterpolation(startPose.getHeading(), bucket.getHeading())
                .build();
        sample11 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(bucket), new Point(sample1Curve), new Point(sample1)))
                .setLinearHeadingInterpolation(bucket.getHeading(), sample1.getHeading())
                .build();
        bucket2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample1), new Point(bucket)))
                .setLinearHeadingInterpolation(sample1.getHeading(), bucket.getHeading())
                .build();
        sample22 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(bucket), new Point(sample2Curve), new Point(sample2)))
                .setLinearHeadingInterpolation(bucket.getHeading(), sample2.getHeading())
                .build();
        bucket3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample2), new Point(bucket)))
                .setLinearHeadingInterpolation(sample2.getHeading(), bucket.getHeading())
                .build();
        sample33 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(bucket), new Point(sample3)))
                .setLinearHeadingInterpolation(bucket.getHeading(), sample3.getHeading())
                .build();
        bucket4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample3), new Point(bucket)))
                .setLinearHeadingInterpolation(sample3.getHeading(), bucket.getHeading())
                .build();
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void autonomousPathUpdate() {

        switch (pathState) {

            case 0:
                lifting();
                follower.followPath(bucket1);
                if (follower.getPose().getX() > (bucket.getX()) && (follower.getPose().getY()) > (bucket.getY())) {
                    dumping();
                    returning();
                    intaking();
                    follower.followPath(bucket1, true);
                    setPathState(1);
                }
            case 1:
                if (follower.getPose().getX() > (sample1.getX() - 1) && (follower.getPose().getY()) > (sample1.getY() - 1)) {
                    Actions.runBlocking(new SleepAction(1));
                    transferring();
                    lifting();
                    follower.followPath(sample11, true);
                    setPathState(2);
                }
            case 2:
                if (follower.getPose().getX() > (bucket.getX() - 1) && (follower.getPose().getY()) > (bucket.getY() - 1)) {
                    dumping();
                    returning();
                    intaking();
                    follower.followPath(bucket2, true);
                    setPathState(3);
                }
            case 3:
                if (follower.getPose().getX() > (sample2.getX() - 1) && (follower.getPose().getY()) > (sample2.getY() - 1)) {
                    Actions.runBlocking(new SleepAction(1));
                    transferring();
                    lifting();
                    follower.followPath(sample22, true);
                    setPathState(4);
                }
            case 4:
                if (follower.getPose().getX() > (bucket.getX() - 1) && (follower.getPose().getY()) > (bucket.getY() - 1)) {
                    dumping();
                    returning();
                    intaking();
                    follower.followPath(bucket3, true);
                    setPathState(5);
                }
            case 5:
                if (follower.getPose().getX() > (sample3.getX() - 1) && (follower.getPose().getY()) > (sample3.getY() - 1)) {
                    Actions.runBlocking(new SleepAction(1));
                    transferring();
                    lifting();
                    follower.followPath(sample33, true);
                    setPathState(6);
                }
            case 6:
                if (follower.getPose().getX() > (bucket.getX() - 1) && (follower.getPose().getY()) > (bucket.getY() - 1)) {
                    dumping();
                    returning();
                    intaking();
                    follower.followPath(bucket4, true);
                    setPathState(-1);
                }
        }
    }


    @Override
    public void runOpMode() {


        follower = new Follower(hardwareMap);


        follower.setStartingPose(startPose);


        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("hi people" + "It's me your Bestie!!!" + "I will not play omori");
        telemetryA.update();


        while (!isStopRequested() && !opModeIsActive()) {

        }


        waitForStart();
        if (isStopRequested()) return;
        follower.update();
        switch (pathState) {
            case 0:
                Actions.runBlocking(
                        new ParallelAction(
                                lift.liftUp(),
                                servoSlide.servoSlideUp()
                        )
                );
                follower.followPath(bucket1);
                if (follower.getPose().getX() > (bucket.getX() - 1) && follower.getPose().getY() > (bucket.getY() - 1)) {
                    Actions.runBlocking(
                            new SequentialAction(
                                    servoDumper.servoDumpDown(),
                                    new SleepAction(1),
                                    servoDumper.servoDumpUp(),
                                    new ParallelAction(
                                            lift.liftDown(),
                                            actuator.intakeOut(),
                                            servoIntake.intakeDown(),
                                            crServoIntake.intakeIn(),
                                            servoSlide.servoSlideDown()
                                    )

                            )
                    );
                    follower.followPath(bucket1, true);

                    pathState = 1;

                }
                break;
        }


        follower.followPath(sample11);
        Actions.runBlocking(new SleepAction(2));
        Actions.runBlocking(new SequentialAction(
                        servoIntake.intakeUp(),
                        actuator.intakeIn(),
                        crServoIntake.intakeOut(),
                        new ParallelAction(
                                lift.liftUp(),
                                servoSlide.servoSlideUp()
                        )
                )
        );
        follower.followPath(bucket2);


    }
}

