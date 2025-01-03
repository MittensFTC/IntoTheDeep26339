package pedroPathing.Mittens4point0;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;




@Autonomous(name = "specimen", group = "Examples")
public class SpecimenAutonomous extends OpMode {

    TeleOperatonalJava robot = new TeleOperatonalJava();

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
                    lift.setTargetPosition(robot.liftingMotorMax);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(0.8);
                    initialized = true;
                }


                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < robot.liftingMotorMax -200) {
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
                if (pos < 200   ) {
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






    private Follower follower;
    private Follower lift;
    private int pathState;

    private final Pose startPose = new Pose(0,0, Math.toRadians(0));
    private final Pose specimen = new Pose(0,30,Math.toRadians(0));
    private final Pose interPose = new Pose(24, -24, Math.toRadians(90));
    private final Pose endPose = new Pose(24, 24, Math.toRadians(45));

    private PathChain triangle;

    private Telemetry telemetryA;


    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void loop() {

        follower.update();
        lift.update();

        if (follower.atParametricEnd()) {
            follower.followPath(triangle, true);
        }


        follower.telemetryDebug(telemetryA);

    }

    /**
     * This initializes the Follower and creates the PathChain for the "triangle". Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        follower = new Follower(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        follower.setStartingPose(startPose);

        triangle = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(interPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), interPose.getHeading())
                .addPath(new BezierLine(new Point(interPose), new Point(endPose)))
                .setLinearHeadingInterpolation(interPose.getHeading(), endPose.getHeading())
                .addPath(new BezierLine(new Point(endPose), new Point(startPose)))
                .setLinearHeadingInterpolation(endPose.getHeading(), startPose.getHeading())
                .build();

        follower.followPath(triangle);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will run in a roughly triangular shape,"
                + "starting on the bottom-middle point. So, make sure you have enough "
                + "space to the left, front, and right to run the OpMode.");
        telemetryA.update();
    }

}
