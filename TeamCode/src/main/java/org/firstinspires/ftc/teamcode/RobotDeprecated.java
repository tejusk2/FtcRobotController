package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RobotDeprecated {
    ServoImplEx OarmLeft, OarmRight, outRight, leftOut;
    DcMotor front_right,front_left,back_left,back_right, enc_left, leftSlides, islides, rightSlides;
    CRServo intakeMotorLeft, intakeMotorRight;
    TouchSensor left, right, intakeLeft, intakeRight;

    RevBlinkinLedDriver leds;
    ElapsedTime profileTimer = new ElapsedTime();
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
    public RobotDeprecated(DcMotor rightFront, DcMotor leftFront, DcMotor rightRear, DcMotor leftRear,
                           DcMotor LeftxAxis, DcMotor leftSlides, DcMotor islides, DcMotor rightSlides,
                           RevBlinkinLedDriver leds,
                           ServoImplEx  OarmLeft, ServoImplEx OarmRight, ServoImplEx outRight, ServoImplEx leftOut, ServoImplEx rightIntake, ServoImplEx leftIntake,
                           TouchSensor left, TouchSensor right,
                           CRServo intakeMotorLeft, CRServo intakeMotorRight) {
        this.front_right = rightFront;
        this.front_left = leftFront;
        this.back_right = rightRear;
        this.back_left = leftRear;
        this.leftSlides = leftSlides;
        this.leds = leds;
        this.OarmLeft = OarmLeft;
        this.OarmRight = OarmRight;
        this.outRight = outRight;
        this.leftOut = leftOut;
        this.left = left;
        this.right = right;
        this.islides = islides;
        this.rightSlides = rightSlides;
        this.intakeMotorLeft = intakeMotorLeft;
        this.intakeMotorRight = intakeMotorRight;

        //enc right = BR
        //enc perp = slides_l
        this.enc_left = LeftxAxis;
        //this.leds = lights;

        //this.leftSlides.setDirection(DcMotorSimple.Direction.REVERSE);
        //back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);
        //front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        enc_left.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    public void timedCreep(double seconds, double power){
        ElapsedTime runTimer = new ElapsedTime();
        runTimer.reset();
        while(runTimer.seconds() < seconds){
            front_right.setPower(power);
            back_left.setPower(power);
            back_right.setPower(power);
            front_left.setPower(power);
        }
        front_right.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
        front_left.setPower(0);
    }
    public void motionProfile(double x, double y){
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
                    leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                    profile_dist = 0.5*max_accel*(Math.pow(profileTimer.seconds(), 2));
                    motionPid(profile_dist, errIntegralSum, kIl, kPl, theta);
                    //DECELERATION
                }else if(profileTimer.seconds() < total_time){
                    leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                    profile_dist = halfway + (adjusted_max_velo*(profileTimer.seconds()-accel_time)) - (0.5*max_accel*(Math.pow((profileTimer.seconds()-accel_time), 2)));
                    motionPid(profile_dist, errIntegralSum,kIl, kPl, theta);
                    //POSITION LOCK
                }else{
                    leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                    profile_dist = total_dist;
                    motionPid(profile_dist, errIntegralSum,kIl, kPl, theta);
                }

            }
            front_right.setPower(0);
            back_left.setPower(0);
            back_right.setPower(0);
            front_left.setPower(0);
        }
        if(accel_distance < halfway){
            profileTimer.reset();
            while(profileTimer.seconds() < total_time + 0.5){
                //ACCEL
                arcLocalizationApprox();
                if(profileTimer.seconds() < accel_time){
                    leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                    profile_dist = 0.5*max_accel*(Math.pow(profileTimer.seconds(), 2));
                    motionPid(profile_dist, errIntegralSum,kIl, kPl,theta);
                    //CRUISE
                }else if(profileTimer.seconds() < accel_time + cruise_time){
                    leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                    profile_dist = accel_distance + (max_velo*(profileTimer.seconds()-accel_time));
                    motionPid(profile_dist, errIntegralSum,kIl, kPl,theta);
                    //DECELERATE
                }else if(profileTimer.seconds() < total_time){
                    leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                    profile_dist = accel_distance + (max_velo*(profileTimer.seconds()-accel_time)) - (0.5*max_accel*(Math.pow((profileTimer.seconds()-(accel_time+cruise_time)), 2)));
                    motionPid(profile_dist, errIntegralSum,kIl, kPl,theta);
                }else{
                    leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                    profile_dist = total_dist;
                    motionPid(profile_dist, errIntegralSum,kIl, kPl,theta);
                }
            }
            front_right.setPower(0);
            back_left.setPower(0);
            back_right.setPower(0);
            front_left.setPower(0);

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

        front_right.setPower((equationOne * pidOutput) + headingPIDOutput);
        back_left.setPower((equationOne * pidOutput) - headingPIDOutput);
        back_right.setPower((equationTwo * pidOutput) + headingPIDOutput);
        front_left.setPower((equationTwo * pidOutput) - headingPIDOutput);


        cycleTime = profileTimer.seconds();
    }
    public void arcLocalizationApprox(){
        x_pos =inchespertickX*(enc_left.getCurrentPosition()+back_right.getCurrentPosition())/2;
        y_pos = leftSlides.getCurrentPosition()*inchespertickY;
        heading = ((back_right.getCurrentPosition()*inchespertickX) - (inchespertickX*enc_left.getCurrentPosition())) / track_width;
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
            leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
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
        back_right.setPower(power);
        back_left.setPower(-power);
        front_left.setPower(-power);
        front_right.setPower(power);
        //arcLocalizationApprox();
    }
    public void resetOdo(){
        enc_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        x_pos = 0;
        y_pos = 0;
        heading = 0;
    }




    public void slidesLimiterOutake(double power, int upperBound, int lowerBound) {
        if ((-rightSlides.getCurrentPosition() > upperBound && power > 0) || (((!left.isPressed() && power < 0) && (!right.isPressed() && power < 0))) || (-rightSlides.getCurrentPosition() < 0 && power < 0)) {
            rightSlides.setPower(0);
            leftSlides.setPower(0);

        } else {
            leftSlides.setPower(power);
            rightSlides.setPower(-power);

        }
    }
    public void slides(double power){
        leftSlides.setPower(power);
        rightSlides.setPower(-power);
    }

    public void specimenDropoffPosition(int pos){

            ElapsedTime timer = new ElapsedTime();
            timer.reset();
            while (timer.seconds() < 1.5) {
                //bring our outtake up
                slidesLimiterOutake(0.8, pos, 0);
                if(operating.isInterrupted()){
                    break;
                }
                if (rightSlides.getPower() == 0) {
                    break;
                }
            }


    }

    public void setOUTTAKE_ARM_DOWN(){
        OarmRight.setPosition(0.00);
        OarmLeft.setPosition(0.00);
    }
    public void setOUTTAKE_ARM_UP(){
        OarmRight.setPosition(1);
        OarmLeft.setPosition(1);
    }
    public void setOUTTAKE_TRANSFER(){
        outRight.setPosition(0.3);
        leftOut.setPosition(0.3);
    }
    public void setOUTTAKE_SPECIMEN(){
        outRight.setPosition(1);
        leftOut.setPosition(1);
    }
    public void setOUTTAKE_DROP(){
        outRight.setPosition(0);
        leftOut.setPosition(0);
    }
    public void disablePWMOuttakeBox(){
        outRight.setPwmDisable();
        leftOut.setPwmDisable();
    }
    public void disablePWMOuttakeArm(){
        OarmRight.setPwmDisable();
        OarmLeft.setPwmDisable();
    }
    //ENABLE
    public void enablePWMOuttakeBox(){
        outRight.setPwmEnable();
        leftOut.setPwmEnable();
    }
    public void enablePWMOuttakeArm(){
        OarmRight.setPwmEnable();
        OarmLeft.setPwmEnable();
    }




}



