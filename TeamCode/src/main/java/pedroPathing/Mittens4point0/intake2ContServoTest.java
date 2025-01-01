package pedroPathing.Mittens4point0;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

public class intake2ContServoTest extends LinearOpMode{

    CRServo servo1 = hardwareMap.crservo.get("servo1");
    CRServo servo2 = hardwareMap.crservo.get("servo2");
    int power = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            waitForStart();
            if (gamepad1.a) {
                power = 1;
            } else if (gamepad1.b){
                power = -1;
            }
            servo1.setPower(power);
            servo2.setPower(-power);
        }
    }

}
