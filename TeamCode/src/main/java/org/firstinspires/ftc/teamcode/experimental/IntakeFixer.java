package org.firstinspires.ftc.teamcode.experimental;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="IntakeFixers")
public class IntakeFixer extends LinearOpMode {
    Limelight3A limelight;
    Servo base, joint, wrist, claw, outtakeBox, outtakeArm;
    double memory = 1;
    @Override
    public void runOpMode() throws InterruptedException {
        base = hardwareMap.get(Servo.class, "base");
        joint = hardwareMap.get(Servo.class, "joint");
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");

        outtakeArm = hardwareMap.get(Servo.class, "outtakeArm");
        outtakeBox = hardwareMap.get(Servo.class, "outtakeBox");

        /*
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        limelight.pipelineSwitch(2); // Switch to pipeline number 1
         */


        waitForStart();
        base.setPosition(0.3);
        joint.setPosition(0.9);
        wrist.setPosition(0);
        claw.setPosition(0);

        outtakeBox.setPosition(0);
        outtakeArm.setPosition(0.225);

        while(opModeIsActive()){
            telemetry.addData("base: ", base.getPosition());
            telemetry.addData("joint: ", joint.getPosition());
            telemetry.addData("wrist: ", wrist.getPosition());
            telemetry.addData("claw: ", claw.getPosition());
            telemetry.addData("oBox: ", outtakeBox.getPosition());
            telemetry.addData("oArm: ", outtakeArm.getPosition());
            telemetry.update();

            //swivel wrist
            if(gamepad1.left_bumper){
                wrist.setPosition(wrist.getPosition()-0.05);
                sleep(200);

            }
            if(gamepad1.right_bumper){
               wrist.setPosition(wrist.getPosition()+0.05);
                sleep(200);
            }

            //arm joint
            if(gamepad1.cross){
                joint.setPosition(joint.getPosition()-0.05);
                sleep(200);
            }
            if(gamepad1.triangle){
                joint.setPosition(joint.getPosition()+0.05);
                sleep(200);

            }
            //outtakeBox
            if(gamepad2.dpad_up){
                outtakeBox.setPosition(outtakeBox.getPosition()+0.05);
                sleep(200);
            }
            if(gamepad2.dpad_down){
                outtakeBox.setPosition(outtakeBox.getPosition()-0.05);
                sleep(200);
            }
            //outtakeArm
            if(gamepad2.triangle){
                outtakeArm.setPosition(outtakeArm.getPosition()+0.05);
                sleep(200);
            }
            if(gamepad2.cross){
                outtakeArm.setPosition(outtakeArm.getPosition()-0.05);
                sleep(200);
            }

            //base
            if(gamepad1.dpad_up){
                base.setPosition(base.getPosition()+0.05);
                sleep(200);
            }
            if(gamepad1.dpad_down){
                base.setPosition(base.getPosition()-0.05);
                sleep(200);
            }
            //claw
            claw.setPosition(0.5 - gamepad1.right_trigger);

            //wrist.setPosition(gamepad1.right_stick_x);
            if(gamepad1.square){
                if(limelight.isRunning()){
                    autoAdjust();
                }
            }

        }

    }
    public void autoAdjust(){
        ElapsedTime runtimer = new ElapsedTime();
        while(runtimer.seconds() < 1.8) {
            double width = findSmallest();
            while (width == 0) {
                width = findSmallest();
            }
            double smallest = width;
            double smallestI = 0;
            boolean cancelled = false;
            for (double i = 0; i <= 1; i += 0.04) {
                wrist.setPosition(i);
                width = findSmallest();
                while (width == 0) {
                    width = findSmallest();
                }
                if (width < smallest) {
                    smallest = width;
                    smallestI = i;
                }
                sleep(30);
                if (gamepad1.circle) {
                    cancelled = true;
                    break;
                }
            }
            if (!cancelled) {
                wrist.setPosition(smallestI - 0.2);
                joint.setPosition(0.8);
                base.setPosition(0.175);
            }
            break;
        }
    }
    public double findSmallest(){
        try {
            if(gamepad1.circle){
                return 1;
            }
            LLResult result = limelight.getLatestResult();

            int smallest = 0;
            int i = 0;
            List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();

            for (LLResultTypes.DetectorResult detection : detections) {
                double x = Math.abs(detection.getTargetXDegrees()); // Where it is (left-right)
                String classname = detection.getClassName();

                i++;
                if (Math.abs(detections.get(smallest).getTargetXDegrees()) > x) {
                    smallest = i;
                }

            }

            double x1 = detections.get(smallest).getTargetCorners().get(0).get(0);
            double x2 = detections.get(smallest).getTargetCorners().get(1).get(0);

            telemetry.addData("x1: ", x1);
            telemetry.addData("x2: ", x2);



            double width = x2-x1;
            memory = width;
            return memory;
        }catch (Exception e){
            return 0;
        }
    }
}