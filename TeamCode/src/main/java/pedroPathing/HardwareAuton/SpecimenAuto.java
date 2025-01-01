package pedroPathing.HardwareAuton;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;


@Config
@Autonomous(name = "Specimen", group = "Autonomous")
public class SpecimenAuto extends LinearOpMode {
    public class Lift {
        private final DcMotorEx lift;




        public Lift(HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotorEx.class, "ma");
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
                    lift.setTargetPosition(2000);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(0.8);
                    initialized = true;
                }




                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 1100.0) {
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
                    lift.setPower(-0.8);
                    initialized = true;
                }




                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 1000.0) {
                    return true;
                } else {
                    lift.setPower(0.3);
                    return false;
                }
            }
        }
        public Action liftDown(){
            return new LiftDown();
        }
    }




//slide


    public  class Slide{
        private DcMotorEx slide;


        public Slide(HardwareMap hardwareMap) {
            slide = hardwareMap.get(DcMotorEx.class, "ls");
            slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slide.setDirection(DcMotorSimple.Direction.REVERSE);
            slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
//play omori




        public class SlideUp implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;


            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    slide.setTargetPosition(300);
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slide.setPower(0.5);
                    initialized = true;
                }


                // checks lift's current position
                double pos = slide.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 250.0) {
                    // true causes the action to rerun
                    packet.put("Rerunning", true);
                    return true;
                } else {
                    // false stops action rerun
                    packet.put("Rerunning", false);


                    slide.setPower(0.8);
                    return false;
                }
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off
            }


        }
        public  Action slideUp() {
            return new SlideUp();
        }
    }
    public class Intake {
        private final CRServo intake;


        public Intake(HardwareMap hardwareMap) {
            intake =  hardwareMap.crservo.get("intake");
        }


        public class intakeOut implements Action {
            int cycles = 0;


            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                packet.put("INTAKE", "ON!!!!");
                intake.setPower(1);


                sleep(1000);
                packet.put("INTAKE", "Off");
                cycles++;
                return cycles < 5;
            }
        }
        public Action intakeOut() {
            return new intakeOut();
        }


        public class intakeIn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake.setPower(-1);
                return false;
            }
        }
        public Action intakeIn() {
            return new intakeIn();
        }
    }














    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Lift lift = new Lift(hardwareMap);
        Slide slide = new Slide (hardwareMap);
        Intake intake = new Intake(hardwareMap);






        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-27,11), Math.toRadians(230));






        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .waitSeconds(4)
                .strafeTo(new Vector2d(60,64));
        Action trajectoryActionCloseOut = tab1.endTrajectory().fresh()
                .strafeTo(new Vector2d(48, 50))
                .build();




        // actions that need to happen on init; for instance, a claw tightening.












        while (!isStopRequested() && !opModeIsActive()) {








        }




        telemetry.update();
        waitForStart();




        if (isStopRequested()) return;










        Actions.runBlocking(
                new SequentialAction(
//                        intake.intakeOut(),
                        lift.liftUp(),
                        tab1.build(),
                        slide.slideUp()




//                       trajectoryActionCloseOut
                )
        );


        Actions.runBlocking(intake.intakeOut());
    }
}





