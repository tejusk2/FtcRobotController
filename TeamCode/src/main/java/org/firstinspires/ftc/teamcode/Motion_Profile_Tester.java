package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.List;

@Autonomous(name="Motion Profile Tester")
public class Motion_Profile_Tester extends LinearOpMode {
    DcMotor fl,fr,br,bl, enc_left, enc_right, enc_x;
    //RevBlinkinLedDriver leds;

    @Override
    public void runOpMode() throws InterruptedException {
        fl = hardwareMap.get(DcMotor.class, "FL");
        bl = hardwareMap.get(DcMotor.class, "BL");
        br = hardwareMap.get(DcMotor.class, "BR");
        fr = hardwareMap.get(DcMotor.class, "FR");

        enc_left = hardwareMap.get(DcMotor.class, "enc_left");
        enc_right = hardwareMap.get(DcMotor.class, "enc_right");
        enc_x = hardwareMap.get(DcMotor.class, "enc_x");
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        //intakeMotorLeft = hardwareMap.get(CRServo.class, "leftIntake");
        //intakeMotorRight = hardwareMap.get(CRServo.class, "rightIntake");
        //leds = hardwareMap.get(RevBlinkinLedDriver.class, "leds");
        //limitSwitch = hardwareMap.get(TouchSensor.class, "limit");
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        enc_left.setDirection(DcMotorSimple.Direction.REVERSE);
        enc_right.setDirection(DcMotorSimple.Direction.REVERSE);
        enc_x.setDirection(DcMotorSimple.Direction.REVERSE);

        Robot robot = new Robot(fr, fl, br, bl, enc_left, enc_x, enc_right);
        encodeReset();

        waitForStart();

        //forward
        robot.motionProfile(0, 70);
        sleep(200);
        //strafe right
        robot.motionProfile(60, 0);
        sleep(200);
        //turn left//counterclockwise
        robot.rotateWithPID(90);
        sleep(200);
        //forward
        robot.motionProfile(0, 60);
        sleep(200);
        //strafe left
        robot.motionProfile(-70, 0);


        /*
        robot.rotateWithPID(90);
        sleep(1000);
        robot.rotateWithPID(-90);
        sleep(1000);
        robot.rotateWithPID(45);
        sleep(1000);
        robot.rotateWithPID(-45);
        sleep(1000);
        robot.rotateWithPID(20);
        sleep(1000);
        robot.rotateWithPID(-20);
         */


    }
    public void encodeReset(){
        enc_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        enc_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        enc_x.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

}
