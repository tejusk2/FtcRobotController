package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Mat;

import java.util.List;

public class ClawSubAssembly {
    Limelight3A limelight;
    public Servo base, joint, wrist, claw;
    public ClawSubAssembly(Limelight3A limelight, Servo base, Servo joint, Servo wrist, Servo claw){
        this.limelight = limelight;
        this.base = base;
        this.joint = joint;
        this.wrist = wrist;
        this.claw = claw;
        try {
            limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
            limelight.start(); // This tells Limelight to start looking
            limelight.pipelineSwitch(2); // Switch to pipeline number 2
        }catch (Exception e){

        }


    }
    public void autoAdjust(){
        ElapsedTime runtimer = new ElapsedTime();
        while(runtimer.seconds() < 1.8) {
            double width = findSmallest();
            while (width == 0 && runtimer.seconds() < 1.8) {
                width = findSmallest();
            }
            double smallest = width;
            double smallestI = 0;
            for (double i = 0; i <= 1; i += 0.04) {
                wrist.setPosition(i);
                width = findSmallest();
                while (width == 0 && runtimer.seconds() < 1.8) {
                    width = findSmallest();
                }
                if (width < smallest) {
                    smallest = width;
                    smallestI = i;
                }
                sleep(30);
            }
            if (runtimer.seconds() < 1.8) {
                wrist.setPosition(smallestI-0.25);
                sleep(100);
                setINTAKE_Middle();
            }
            break;
        }
    }
    public double findSmallest(){
        try {
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

            return x2-x1;

        }catch (Exception e){
            return 0;
        }
    }
    public boolean linedUp(){
        try {
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

            double centerX = result.getTx();
            if(Math.abs(centerX) < 15){
                return true;
            }else{
                return false;
            }
        }catch (Exception e){
            return false;
        }
    }
    public void sleep(int milliseconds){
        ElapsedTime sleepTimer = new ElapsedTime();
        sleepTimer.reset();
        while(sleepTimer.milliseconds() <= milliseconds){
            continue;
        }
    }
    public void setINTAKE_SCAN(){
        base.setPosition(0.4);
        joint.setPosition(0.95);
        wrist.setPosition(0);
    }
    public void setINTAKE_TRANSFER(){
        wrist.setPosition(0);
        joint.setPosition(0);
        base.setPosition(0.25);
    }
    public void setINTAKE_PICKUP(){
        joint.setPosition(0.925);
        base.setPosition(0.16);
        openClaw();
    }
    public void setINTAKE_Middle(){
        joint.setPosition(0.94);
        base.setPosition(0.26);
        openClaw();
    }
    public void setINTAKE_WALL(){
        wrist.setPosition(0);
        joint.setPosition(0.75);
        base.setPosition(0.5);
        openClaw();
    }
    public void setINTAKE_WALL_UP(){
        wrist.setPosition(0);
        joint.setPosition(0.50);
        base.setPosition(0.5);
    }

    public void closeClaw(){
        claw.setPosition(0);
    }
    public void openClaw(){
        claw.setPosition(0.7);
    }

}
