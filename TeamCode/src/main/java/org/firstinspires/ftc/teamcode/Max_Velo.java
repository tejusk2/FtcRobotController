package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.List;


@TeleOp(name="max velocity tuner")
public class Max_Velo extends LinearOpMode {

    //USE THIS TO FIND MAX VELOCITY

    IMU imu;
    DcMotor back_right, back_left, front_right, front_left, enc_x, enc_right, enc_left;

    private double inchespertickX = Robot.inchespertickX;
    private double inchespertickY = Robot.inchespertickY;

    int i = 1;
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

        waitForStart();
        encodeReset();
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        int prev_right=0;
        int prev_left=0;
        double start_time = 0;
        double end_time = 0;
        double velo = 0;
        while(opModeIsActive()){
            while(timer.seconds() < 1.5){
                telemetry.addData("encoder right: ", enc_right.getCurrentPosition());
                telemetry.addData("encoder left: ", enc_left.getCurrentPosition());
                telemetry.update();

                back_right.setPower(1);
                back_left.setPower(1);
                front_left.setPower(1);
                front_right.setPower(1);

                if(timer.seconds() > 0.5 && prev_left== 0){
                    prev_left = enc_left.getCurrentPosition();
                    prev_right = enc_right.getCurrentPosition();
                    start_time = timer.seconds();
                }
                if(timer.seconds() > 1 && end_time== 0){

                    end_time = timer.seconds();
                    double currentPos = (enc_left.getCurrentPosition() + enc_right.getCurrentPosition())/2;
                    double startPos = (prev_left + prev_right)/2;
                    velo = (currentPos - startPos) / (end_time - start_time);
                }
            }
            telemetry.addData("max_velo: ", velo);
            telemetry.update();
            back_right.setPower(0);
            back_left.setPower(0);
            front_left.setPower(0);
            front_right.setPower(0);

            sleep(6000);


        }

    }
    public void encodeReset(){
        enc_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        enc_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        enc_x.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
}
