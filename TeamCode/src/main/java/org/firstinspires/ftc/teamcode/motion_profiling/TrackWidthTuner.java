package org.firstinspires.ftc.teamcode.motion_profiling;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.RobotDeprecated;

import java.util.List;

@TeleOp(name="track width tuner")
public class TrackWidthTuner extends LinearOpMode {

    DcMotor back_right, back_left, front_right, front_left, enc_x, enc_right, enc_left;

    private double inchespertickX = RobotDeprecated.inchespertickX;
    private double inchespertickY = RobotDeprecated.inchespertickY;

    int i = 1;

    //THIS PROGRAM WILL ATTEMPT TO TURN 90 DEGREES
    //TUNE THE TRACK WIDTH TO MAKE IT TURN 90

    @Override
    public void runOpMode() throws InterruptedException {
        back_right = hardwareMap.get(DcMotor.class,"BR");
        back_left = hardwareMap.get(DcMotor.class,"BL");
        front_right = hardwareMap.get(DcMotor.class,"FR");
        front_left = hardwareMap.get(DcMotor.class,"FL");

        enc_x = hardwareMap.get(DcMotor.class,"enc_x");
        enc_right = hardwareMap.get(DcMotor.class,"enc_right");
        enc_left = hardwareMap.get(DcMotor.class,"enc_left");

        enc_x.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        enc_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        enc_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);
        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        enc_left.setDirection(DcMotorSimple.Direction.REVERSE);
        enc_right.setDirection(DcMotorSimple.Direction.REVERSE);
        enc_x.setDirection(DcMotorSimple.Direction.REVERSE);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        double track_width = 10.7445843;
        encodeReset();
        waitForStart();

        while(opModeIsActive()){
            double theta = ((enc_left.getCurrentPosition()*inchespertickX) - (enc_right.getCurrentPosition()*inchespertickX)) / track_width;
            while(Math.abs(Math.toDegrees(theta)) < 90*i){

                telemetry.addData("predicted total change in heading: ", Math.toDegrees(theta));
                telemetry.addData("track width: ", track_width);
                telemetry.addData("encoder right: ", enc_right.getCurrentPosition());
                telemetry.addData("encoder left: ", enc_left.getCurrentPosition());
                telemetry.update();

                back_right.setPower(-0.2);
                back_left.setPower(0.2);
                front_left.setPower(0.2);
                front_right.setPower(-0.2);
                theta = ((enc_left.getCurrentPosition()*inchespertickX) - (enc_right.getCurrentPosition()*inchespertickX)) / track_width;
            }
            back_right.setPower(0);
            back_left.setPower(0);
            front_left.setPower(0);
            front_right.setPower(0);
            while(true){
                if(gamepad1.cross){
                    i+=1;
                    break;
                }
                if(gamepad1.circle){
                    track_width += 0.01;
                    sleep(100);
                }
                if(gamepad1.square){
                    track_width -= 0.01;
                    sleep(100);
                }
                telemetry.addData("track_width: ", track_width);
                telemetry.update();
            }

        }

    }
    public void encodeReset(){
        enc_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        enc_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        enc_x.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
}
