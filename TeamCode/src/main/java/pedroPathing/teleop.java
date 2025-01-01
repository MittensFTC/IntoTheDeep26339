package pedroPathing;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;



@TeleOp
public class teleop extends LinearOpMode {


    int desiredPosition;
    int slidePosition;
    public double speed = 1;


    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("leftFront");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("leftBack");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("rightFront");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("rightBack");
        DcMotor motorArm = hardwareMap.dcMotor.get("ma");
        DcMotor linearSlide = hardwareMap.dcMotor.get("ls");
        CRServo intake = hardwareMap.crservo.get("intake");


        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        int armPosition;


        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        desiredPosition = 0;
        waitForStart();
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidePosition = 0;


        if (isStopRequested()) return;

        while (opModeIsActive()) {
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


            if (gamepad2.y){
                desiredPosition = desiredPosition + 8;
            }
            else if (gamepad2.x) {
                desiredPosition = desiredPosition - 8;

            }
            else{
                desiredPosition = desiredPosition;
            }


            motorArm.setTargetPosition(desiredPosition);
            motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorArm.setPower(0.75);








            if (gamepad2.a){
                slidePosition = slidePosition + 8;
            }
            else if (gamepad2.b) {
                slidePosition = slidePosition - 8;

            }
            else{
                slidePosition = slidePosition;
            }


            linearSlide.setTargetPosition(slidePosition);
            linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linearSlide.setPower(0.75);




            if (gamepad2.right_bumper){
                intake.setPower(1);
            } else if (gamepad2.left_bumper) {
                intake.setPower(-1);
            }
            else{
                intake.setPower(0);
            }


            telemetry.addData("Slide", linearSlide.getCurrentPosition());
            telemetry.update();














            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx)*speed / denominator;
            double backLeftPower = (y - x + rx)*speed / denominator;
            double frontRightPower = (y - x - rx)*speed / denominator;
            double backRightPower = (y + x - rx)*speed / denominator;








            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);
        }
    }
}



