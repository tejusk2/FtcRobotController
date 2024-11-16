package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.BatteryChecker;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.List;

@TeleOp(name="rotational PID")
public class RotationalPID extends LinearOpMode {

    DcMotor back_right, back_left, front_right, front_left, enc_x, enc_right, enc_left;

    private double inchespertickX = Robot.inchespertickX;
    private double inchespertickY = Robot.inchespertickY;

    int kP = 11;
    int kD = 1;
    int kI = 2;

    double track_width = Robot.track_width;
    int i = 1;

    //HOW TO TUNE:
    //SET KD AND KI TO 0
    //increase kP until the robot is oscillating on it's target pos, then half that
    //increase kI until the robot quickly reaches target pos
    //increase kD to make the robot more responsive to changes in error, really not needed IMO


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
        while(opModeIsActive()){
            while(true){
                if(gamepad1.cross){
                    break;
                }
                if(gamepad1.circle){
                    PID(90);
                }
                if(gamepad1.square){
                    PID(-90);
                }
                telemetry.addData("kP: ", kP);
                telemetry.addData("kD: ", kD);
                telemetry.addData("kI: ", kI);
                telemetry.update();
                if(gamepad1.right_bumper){
                    kI+=1;
                    sleep(100);
                }
                if(gamepad1.left_bumper){
                    kI-=1;
                    sleep(100);
                }

            }

        }

    }


    public void PID(double deltaTheta){
        double theta = ((enc_left.getCurrentPosition()*inchespertickX) - (enc_right.getCurrentPosition()*inchespertickX)) / track_width;

        //well run our PID until our output becomes super small
        double lastError = 0;
        ElapsedTime cycleTimer = new ElapsedTime();
        double integralSum = 0;
        while(true){
            theta = ((enc_left.getCurrentPosition()*inchespertickX) - (enc_right.getCurrentPosition()*inchespertickX)) / track_width;
            double error = deltaTheta - Math.toDegrees(theta);
            double deltaError = error - lastError;
            double proportional = kP*error;
            double derivative = (deltaError)/cycleTimer.time()*kD;
            integralSum += (error * cycleTimer.time());
            cycleTimer.reset();
            double integral = integralSum*kI;
            //apply windup clamp
            integral *= integralWindupClamp(integral, error);

            double output = (proportional + integral + derivative) / (Math.abs(deltaTheta)*10);

            //need to abs this output because it could be negative
            if(Math.abs(output) < 0.01){
                break;
            }
            back_right.setPower(-output);
            back_left.setPower(output);
            front_left.setPower(output);
            front_right.setPower(-output);

            lastError = error;
            telemetry.addData("predicted total change in heading: ", Math.toDegrees(theta));
            telemetry.addData("kP: ", kP);
            telemetry.addData("kD: ", kD);
            telemetry.addData("kI: ", kI);
            telemetry.update();
            if(gamepad1.right_bumper){
                kI+=1;
                sleep(200);
            }
            if(gamepad1.left_bumper){
                kI-=1;
                sleep(200);
            }

        }
        back_right.setPower(0);
        back_left.setPower(0);
        front_left.setPower(0);
        front_right.setPower(0);
        enc_x.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        enc_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        enc_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(1000);

    }

    public int integralWindupClamp(double integral, double error){
        //if integral term is in opposite direction of error(accelerating the wrong way), zero the integral term
        if(integral / Math.abs(integral) == error / Math.abs(error)){
            return 1;
        }else{
            return 0;
        }

    }
    public void encodeReset(){
        enc_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        enc_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        enc_x.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
}
