package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.lang.reflect.Method;
import java.util.concurrent.Callable;

public class Robot {
    DcMotor fl,fr,br,bl, oSlideRight, oSlideLeft, islide;
    Deadwheel leftX, middle, rightX;
    public ClawSubAssembly intakeArm;
    ServoImplEx outtakeBox, outtakeArm, claw, base, wrist, joint ;
    IMU imu;
    RevBlinkinLedDriver leds;
    TouchSensor left, right, intakeLeft, intakeRight;    //34.1632
    LimitSwitch oLSLeft, oLSRight, iLSLeft,iLSRight;
    ElapsedTime profileTimer = new ElapsedTime();
    ElapsedTime transferTimer = new ElapsedTime();
    Limelight3A limelight;
    double deltaPosLeft = 0;
    double deltaPosRight = 0;
    double deltaPosPerp = 0;

    double prev_posLeft = 0;
    double prev_posRight = 0;
    double prev_posPerp = 0;

    private double x_pos;
    private double y_pos;
    private double heading;

    Thread operating = new Thread();

    double cycleTime = 0;

    enum TransferStates {
        ELSEWHERE,
        TRANSFER_READY,
        SAMPLE_DROPPED
    }
    Tele25.TransferStates transferState = Tele25.TransferStates.ELSEWHERE;



    public static double inchespertickX = 0.00048741851;
    public static double inchespertickY = 0.000482175;

    //double max_velo = 92499.20743 * inchespertickX;
    double max_velo = 100000 * inchespertickX;

    double max_accel = max_velo / 0.5;


    //SUBSTITUTE THIS VALUE FROM ROTATIONAL PID
    int kP = 11;
    int kD = 1;
    int kI = 2;
    double lastError = 0;
    ElapsedTime cycleTimer = new ElapsedTime();
    double integralSum = 0;

    //SUBSTITUTE THIS VALUE FROM TRACK WIDTH TUNER
    public static double track_width = 10.7454843;
    public Robot(DcMotor fr, DcMotor fl, DcMotor br, DcMotor bl,
                 DcMotor LeftxAxis, DcMotor leftSlides, DcMotor islides, DcMotor rightSlides,
                 RevBlinkinLedDriver leds,
                 ServoImplEx outtakeArm, ServoImplEx outtakeBox,
                 ServoImplEx claw, ServoImplEx base, ServoImplEx joint, ServoImplEx wrist,
                 TouchSensor left, TouchSensor right, TouchSensor iLeft, TouchSensor iRight, Limelight3A limelight) {
        this.fr = fr;
        this.fl = fl;
        this.br = br;
        this.bl = bl;

        this.oSlideLeft = leftSlides;
        this.leds = leds;

        this.outtakeArm = outtakeArm;
        this.outtakeBox = outtakeBox;

        this.islide = islides;
        this.oSlideRight = rightSlides;

        this.left = left;
        this.right = right;
        this.intakeLeft = iLeft;
        this.intakeRight = iRight;

        oLSLeft = new LimitSwitch(left);
        oLSRight = new LimitSwitch(right);
        iLSLeft = new LimitSwitch(intakeLeft);
        iLSRight = new LimitSwitch(intakeRight);
        this.limelight = limelight;
        /*
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        limelight.pipelineSwitch(2); // Switch to pipeline number 1
         */
        this.claw = claw;
        this.wrist = wrist;
        this.base  = base;
        this.joint = joint;
        intakeArm = new ClawSubAssembly(limelight, base, joint, wrist, claw);


        LeftxAxis.setDirection(DcMotorSimple.Direction.REVERSE);
        leftX = new Deadwheel(LeftxAxis);
        rightX = new Deadwheel(br);
        middle = new Deadwheel(leftSlides);



        br.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



    }
    public void timedCreep(double seconds, double power){
        ElapsedTime runTimer = new ElapsedTime();
        runTimer.reset();
        while(runTimer.seconds() < seconds){
            fr.setPower(power);
            bl.setPower(power);
            br.setPower(power);
            fl.setPower(power);
        }
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        fl.setPower(0);
    }
    public void motionProfile(double x, double y, double fix_time){
        resetOdo();
        double accel_time = max_velo / max_accel;
        double accel_distance = 0.5*max_accel*(Math.pow(accel_time,2));
        double total_dist = Math.sqrt((x*x)+(y*y));
        double cruise_time  = ((total_dist - (2*accel_distance))/max_velo);
        double total_time = cruise_time + (accel_time*2);
        double halfway = total_dist / 2;
        //PID CONSTANTS
        //Need to tune these
        double kPl = 6;
        double kIl = 1;
        double errIntegralSum = 0;
        cycleTime = 0;
        double profile_dist;
        //TRAVERSAL ANGLE
        double theta = Math.atan2(y, x);
        //if we cant accelerate to max velocity by the distance halfway point
        rotationalPIDSetup();
        if(accel_distance > halfway){
            accel_time = Math.sqrt((2*halfway) / max_accel);
            total_time = accel_time*2;
            double adjusted_max_velo = max_accel * accel_time;
            profileTimer.reset();
            while(profileTimer.seconds() < total_time + 0.5){
                //ACCELERATION
                arcLocalizationApprox();
                if(profileTimer.seconds() < accel_time){
                    if(leds!=null){
                        leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                    }
                    profile_dist = 0.5*max_accel*(Math.pow(profileTimer.seconds(), 2));
                    motionPid(profile_dist, errIntegralSum, kIl, kPl, theta);
                    //DECELERATION
                }else if(profileTimer.seconds() < total_time){
                    if(leds!=null) {
                        leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                    }
                    profile_dist = halfway + (adjusted_max_velo*(profileTimer.seconds()-accel_time)) - (0.5*max_accel*(Math.pow((profileTimer.seconds()-accel_time), 2)));
                    motionPid(profile_dist, errIntegralSum,kIl, kPl, theta);
                    //POSITION LOCK
                }else{
                    if(leds!=null) {
                        leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                    }
                    profile_dist = total_dist;
                    motionPid(profile_dist, errIntegralSum,kIl, kPl, theta);
                }

            }
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);
            fl.setPower(0);
        }
        if(accel_distance < halfway){
            profileTimer.reset();
            while(profileTimer.seconds() < total_time + fix_time){
                //ACCEL
                arcLocalizationApprox();
                if(profileTimer.seconds() < accel_time){
                    if(leds!=null) {
                        leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                    }
                    profile_dist = 0.5*max_accel*(Math.pow(profileTimer.seconds(), 2));
                    motionPid(profile_dist, errIntegralSum,kIl, kPl,theta);
                    //CRUISE
                }else if(profileTimer.seconds() < accel_time + cruise_time){
                    if(leds!=null) {
                        leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                    }
                    profile_dist = accel_distance + (max_velo*(profileTimer.seconds()-accel_time));
                    motionPid(profile_dist, errIntegralSum,kIl, kPl,theta);
                    //DECELERATE
                }else if(profileTimer.seconds() < total_time){
                    if(leds!=null) {
                        leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                    }
                    profile_dist = accel_distance + (max_velo*(profileTimer.seconds()-accel_time)) - (0.5*max_accel*(Math.pow((profileTimer.seconds()-(accel_time+cruise_time)), 2)));
                    motionPid(profile_dist, errIntegralSum,kIl, kPl,theta);
                }else{
                    if(leds!=null) {
                        leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                    }
                    profile_dist = total_dist;
                    motionPid(profile_dist, errIntegralSum,kIl, kPl,theta);
                }
            }
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);
            fl.setPower(0);

        }

        resetOdo();


    }

    public void motionPid(double profile_dist, double errIntegralSum, double kIl, double kPl, double travel_heading){
        double strafe_err = (profile_dist*Math.cos(travel_heading)) - y_pos;
        double fwd_err = (profile_dist*Math.sin(travel_heading)) - x_pos;

        double theta = joystickNormalize(Math.atan2(fwd_err, -strafe_err)) % (Math.PI*2);
        double equationOne = (Math.sin(theta + (Math.PI / 4))); //Front-Right and Back-Left
        double equationTwo = (Math.cos(theta + (Math.PI / 4))); //Front-Left and Back-Right
        //PID calculations
        double headingPIDOutput = rotationalPID(0, 1, 0, 0);
        double posError = Math.sqrt((strafe_err*strafe_err)+(fwd_err*fwd_err));
        double delta_time = profileTimer.seconds() - cycleTime;
        errIntegralSum += (posError*delta_time);
        double integral = errIntegralSum * integralWindupClamp(errIntegralSum, posError) * kIl;
        double proportional = posError * kPl;
        double pidOutput = (integral + proportional) / 100;

        fr.setPower((equationOne * pidOutput) + headingPIDOutput);
        bl.setPower((equationOne * pidOutput) - headingPIDOutput);
        br.setPower((equationTwo * pidOutput) + headingPIDOutput);
        fl.setPower((equationTwo * pidOutput) - headingPIDOutput);


        cycleTime = profileTimer.seconds();
    }
    public void arcLocalizationApprox(){
        x_pos =inchespertickX*(leftX.getCurrentPosition()+rightX.getCurrentPosition())/2;
        y_pos = middle.getCurrentPosition()*inchespertickY;
        heading = ((rightX.getCurrentPosition()*inchespertickX) - (inchespertickX*leftX.getCurrentPosition())) / track_width;
    }
    public double joystickNormalize(double theta){
        theta = (theta >= 0 && theta < Math.toRadians(270)) ? (-1 * theta) + Math.toRadians(90) : (-1 * theta) + Math.toRadians(450);
        theta = (theta < 0) ? Math.toRadians(360) + theta : theta;
        return theta;
    }
    public double rotationalPID(double deltaTheta, double kP, double kI, double kD){

        //run our PID until our output becomes super small
        arcLocalizationApprox();
        double error = deltaTheta - Math.toDegrees(heading);
        double deltaError = error - lastError;
        double proportional = kP*error;
        double derivative = (deltaError)/cycleTimer.time()*kD;
        integralSum += (error * cycleTimer.time());
        cycleTimer.reset();
        double integral = integralSum*kI;
        //apply windup clamp
        integral *= integralWindupClamp(integral, error);

        double output = (proportional + integral + derivative) / 200;
        //need to abs this output because it could be negative
        if(Math.abs(output) < 0.01){
            return 0;
        }

        lastError = error;
        return output;
    }
    public void rotationalPIDSetup(){
        cycleTimer.reset();
        integralSum = 0;
        lastError = 0;
    }
    public int integralWindupClamp(double integral, double error){
        //if integral term is in opposite direction of error(accelerating the wrong way), zero the integral term
        if(Math.signum(integral) == Math.signum(error)){
            return 1;
        }else{
            return 0;
        }

    }
    public void rotateWithPID(double deltaTheta){
        heading = 0;
        rotationalPIDSetup();
        arcLocalizationApprox();
        ElapsedTime rotateTimer = new ElapsedTime();
        rotateTimer.reset();
        while(rotateTimer.seconds() < 1.5){
            if(leds!=null) {
                leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
            }
            double power = rotationalPID(deltaTheta, 2, 1, 0);
            if(power == 0){
                break;
            }
            turn(power);
        }
        turn(0);
        resetOdo();
    }
    public void turn(double power){
        br.setPower(power);
        bl.setPower(-power);
        fl.setPower(-power);
        fr.setPower(power);
    }
    public void resetOdo(){
        leftX.reset();
        rightX.reset();
        middle.reset();

        x_pos = 0;
        y_pos = 0;
        heading = 0;
    }


    public void slidesLimiterIntake(double power, int upperBound){
        if((islide.getCurrentPosition() > upperBound && power > 0) || (intakeDocked() && power < 0)) {
            islide.setPower(0);
        } else{
            islide.setPower(power);
        }
    }
    public void setIntakeSlides(int reference) {
       double output = 1;
       int k = 0;
       ElapsedTime runTimer = new ElapsedTime();
       runTimer.reset();
       while(output > 0.05+k) {
           if(runTimer.seconds() > 3){break;}
           int kP = 1;
           int error = reference - islide.getCurrentPosition();
           output = ((error / 100) + k)*kP;
           slidesLimiterIntake(output, 1300);
       }
    }
    public void homeIntake(){
        while(!intakeDocked()){
            slidesLimiterIntake(-0.9, 1000);
        }
    }
    public void homeOuttake(){
        while(!outtakeDocked()){
            slidesLimiterOutake(-0.9, 1300);
        }
    }
    public void setOuttakeSlides(int reference) {
        double output = 1;
        int k = 0;
        ElapsedTime runTimer = new ElapsedTime();
        runTimer.reset();
        while(output > 0.05+k) {
            if(runTimer.seconds() > 3){break;}
            int kP = 1;
            int error = reference - -1*oSlideRight.getCurrentPosition();
            output = ((error / 100) + k)*kP;
            slidesLimiterOutake(output, 1000);
        }

    }
    public void slidesLimiterOutake(double power, int upperBound) {
        if ((-oSlideRight.getCurrentPosition() > upperBound && power > 0) && (outtakeDocked() && power < 0)){
            oSlideRight.setPower(0);
            oSlideLeft.setPower(0);

        } else {
            oSlideLeft.setPower(power);
            oSlideRight.setPower(-power);

        }
    }
    public void transfer() {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while(!intakeDocked() && timer.seconds() < 3){
            homeIntake();
        }
        while(!outtakeDocked() && timer.seconds() < 3){
            homeOuttake();
        }
        if(intakeDocked() && outtakeDocked()){
            intakeArm.setINTAKE_TRANSFER();
            setOUTTAKE_ARM_TRANSFER();
            setOUTTAKE_TRANSFER();
            timer.reset();
            while(timer.seconds() < 0.75){
                continue;
            }
            intakeArm.openClaw();
            timer.reset();
            while(timer.seconds() < 0.5){
                continue;
            }
            intakeArm.setINTAKE_SCAN();
        }

    }
    public boolean intakeDocked(){
        if((iLSLeft.isPressed() && iLSRight.isPressed()) || islide.getCurrentPosition() <= 0){
            islide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            islide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            return true;
        }else{
            return false;
        }
    }
    public boolean outtakeDocked(){
        if((oLSRight.isPressed() && oLSLeft.isPressed()) || -oSlideRight.getCurrentPosition() <= 0){
            oSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            oSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            return true;
        }else{
            return false;
        }
    }


    public void delayedFunc(double delay, Runnable function){
        new Thread(()->{
            ElapsedTime timer = new ElapsedTime();
            timer.reset();
            while(timer.seconds() <= delay){
                continue;
            }
            function.run();
        }).start();
    }



    //SERVO_POSES
    public void setOUTTAKE_ARM_DOWN(){
        outtakeArm.setPosition(0);
    }
    public void setOUTTAKE_ARM_UP(){
        outtakeArm.setPosition(1);
    }
    public void setOUTTAKE_ARM_TRANSFER(){
        outtakeArm.setPosition(0.225);
    }
    public void setOUTTAKE_ARM_MIDDLE(){
        outtakeArm.setPosition(0.4);
    }
    public void setOUTTAKE_TRANSFER(){
        outtakeBox.setPosition(0.025);
    }
    public void setOUTTAKE_SPECIMEN(){
        outtakeBox.setPosition(1);
    }
    public void setOUTTAKE_DROP(){
        outtakeBox.setPosition(0);
    }




}



