package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.List;
@TeleOp(name = "tele")
public class NewTele extends LinearOpMode {
    Limelight3A limelight;
    DcMotor fl,fr,br,bl;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        limelight.pipelineSwitch(1); // Switch to pipeline number 1

        fl = hardwareMap.get(DcMotor.class, "motor0");
        bl = hardwareMap.get(DcMotor.class, "motor1");
        br = hardwareMap.get(DcMotor.class, "motor2");
        fr = hardwareMap.get(DcMotor.class, "motor3");

        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();

        while (opModeIsActive()){
            if(gamepad1.cross){
                autoAdjust();
            }
        }

    }


    public void autoAdjust(){

        double Xvalue = findSmallest();
        while(Math.abs(Xvalue)>0){
            fl.setPower(0.5);
            bl.setPower(0.5);
            fr.setPower(-0.5);
            br.setPower(-0.5);

            Xvalue = findSmallest();
        }
        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);



    }

    public double findSmallest(){
        LLResult result = limelight.getLatestResult();

        int smallest = 0;
        int i = 0;
        List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
        for (LLResultTypes.DetectorResult detection : detections) {
            String className = detection.getClassName(); // What was detected
            double x = Math.abs(detection.getTargetXDegrees()); // Where it is (left-right)
            i ++;
            if(Math.abs(detections.get(smallest).getTargetXDegrees()) > x){
                smallest = i;
            }
        }
        return detections.get(smallest).getTargetXDegrees();
    }
}