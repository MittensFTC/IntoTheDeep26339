package pedroPathing.Mittens4point0;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoSetPosition extends LinearOpMode {
    Servo servo = hardwareMap.servo.get("servo");

    @Override
    public void runOpMode() throws InterruptedException {
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            waitForStart();
            double pos = servo.getPosition();
            if (gamepad1.a){
                pos += 0.01;
            } else if (gamepad1.b){
                pos -= 0.01;
            }
            telemetry.addData("Servo: ", servo.getPosition());
            telemetry.update();
        }
    }
}
