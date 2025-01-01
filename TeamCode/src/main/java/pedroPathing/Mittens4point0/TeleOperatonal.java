package pedroPathing.Mittens4point0;




import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class TeleOperatonal extends LinearOpMode {

    double speed = 0.75;
    int intakeMotorMin = 0;
    int intakeMotorMax = 2000; //replace with empirical value for max encoder pos
    int liftingMotorMin = 0;
    int liftingMotorMax = 3000; //replace with empirical value for max encoder pos
    int specimenMotorMin = 0;
    int specimenMotorMax = 2000;//replace with empirical value for max encoder pos
    int intakePower = 1;
    double servoIntakeUpPos; //replace with empirical value for intake starting pos
    double servoIntakeDownPos; //replace with empirical value for intake intaking pos
    double servoSlideDownPos; //replace with empirical value for dumper starting pos on slide
    double servoSlideUpPos; //replace with empirical value for dumper dumping pos on slide
    double specimenservo1OpenPos;
    double specimenservo1ClosedPos;
    double specimenservo2OpenPos;
    double specimenservo2ClosedPos;
    double servoDumperUpPos;
    double servoDumperDownPos;
    public Servo servoSlide;
    public Servo servoIntake;
    public Servo servospecimen1;
    public Servo servospecimen2;
    public CRServo intake1;
    DcMotor motorIntake;
    DcMotor motorUp;
    DcMotor motorSpecimen;
    public Servo servoDumper;





    @Override
    public void runOpMode() throws InterruptedException {

        servoDumper = hardwareMap.servo.get("sd");
        servoSlide = hardwareMap.servo.get("ss");
        servoIntake = hardwareMap.servo.get("si");
        servospecimen1 = hardwareMap.servo.get("ss1");
        servospecimen2 = hardwareMap.servo.get("ss2");
        intake1 = hardwareMap.crservo.get("si1");
        motorIntake = hardwareMap.dcMotor.get("mi");
        motorUp = hardwareMap.dcMotor.get("mu");
        motorSpecimen = hardwareMap.dcMotor.get("ms");
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("mfl");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("mfr");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("mbl");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("mbr");

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorSpecimen.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSpecimen.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorUp.setTargetPosition(0);
        motorIntake.setTargetPosition(0);
        motorSpecimen.setTargetPosition(0);
        motorUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorIntake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorSpecimen.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorUp.setPower(1);
        motorIntake.setPower(1);
        motorSpecimen.setPower(1);
        servoSlide.setPosition(servoSlideDownPos);
        servoIntake.setPosition(servoIntakeUpPos);
        intake1.setPower(0);
        servoDumper.setPosition(servoDumperDownPos);
        int step = 0;



        if (isStopRequested()) return;
        while (opModeIsActive()) {
            waitForStart();


            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            if (gamepad1.dpad_up){
                speed = 1;
            }
            if (gamepad1.dpad_right){
                speed = 0.75;
            }
            if (gamepad1.dpad_down){
                speed = 0.5;
            }
            if (gamepad1.dpad_left){
                speed = 0.25;
            }
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx)*speed / denominator;
            double backLeftPower = (y - x + rx)*speed / denominator;
            double frontRightPower = (y - x - rx)*speed / denominator;
            double backRightPower = (y + x - rx)*speed / denominator;
            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);
            switch(step){
                case 0:
                    if (gamepad1.a){step = 1;break;}
                case 1:
                    intaking();
                    if (gamepad1.b){step = 2;break;}
                    else if (gamepad1.x){step = 6;}
                case 2:
                    transferring();
                    sleep(1000);
                    step =3;
                case 3:
                    lifting();
                    if (gamepad1.b){step = 4;break;}
                case 4:
                    dumping();
                    step = 5;
                case 5:
                    returning();
                    step = 0;
                case 6:
                    cancellation();
                    sleep(1000);
                    step = 0;
            }




        }
    }
    public void intaking(){
        motorIntake.setTargetPosition(intakeMotorMax);
        motorIntake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        servoIntake.setPosition(servoIntakeDownPos);
        intake1.setPower(intakePower);

    }

    public void transferring(){
        motorIntake.setTargetPosition(intakeMotorMin);
        motorIntake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        servoIntake.setPosition(servoIntakeUpPos);
        if (motorIntake.getTargetPosition() == intakeMotorMin && servoIntake.getPosition() == servoIntakeUpPos){
            intake1.setPower(-intakePower);
        }

    }

    public void lifting(){
        motorUp.setTargetPosition(liftingMotorMax);
        motorUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(500);
        servoSlide.setPosition(servoSlideUpPos);
        servoDumper.setPosition(servoIntakeDownPos);
    }

    public void dumping(){
        servoDumper.setPosition(servoDumperDownPos);
        sleep(1000);
        servoDumper.setPosition(servoDumperUpPos);
        servoSlide.setPosition(servoIntakeDownPos);
    }

    public void returning(){

        motorUp.setTargetPosition(liftingMotorMin);
        motorUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void cancellation(){
        motorIntake.setTargetPosition(intakeMotorMax);
        sleep(500);
        intake1.setPower(1);
    }

    public void specimenPick(){}

    public void specimenPut(){}

    public void hang(){}
}