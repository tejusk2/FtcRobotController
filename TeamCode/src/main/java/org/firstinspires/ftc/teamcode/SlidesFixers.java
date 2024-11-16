package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Mat;

import java.util.List;

@TeleOp(name="SlidesFixers")
public class SlidesFixers extends LinearOpMode {
    DcMotor fl,fr,br,bl, oSlideRight, oSlideLeft, islide;

    Servo outRight, leftOut, rightIntake, leftIntake, OarmLeft, OarmRight;


    CRServo intakeMotorLeft, intakeMotorRight;
    //RevBlinkinLedDriver leds;
    TouchSensor limitSwitch;
    //34.1632
    double track_width = Robot.track_width;

    double inchesPerTickX = Robot.inchespertickX;
    double inchesPerTickY = Robot.inchespertickY;

    ElapsedTime cycleTimer = new ElapsedTime();
    double lastError = 0;
    double integralSum = 0;
    double kP = 4, kI = 0, kD = 2;

    //servo constants -- for quick use
    final double OUTTAKE_TRANSFER = 0.2, SPECIMEN = 0.5, SAMPLE_DROP = 0.7, OUTTAKE_LEVEL = 0;
    final double INTAKE_TRANSFER = 1, INTAKE_LEVEL = 0.8, PICKUP = 0.3;
    final double OUTTAKE_ARM_UP = 1, OUTTAKE_ARM_DOWN = 0;



    enum DrivingStates {
        FULL_SPEED_MANUAL,
        HEADINGLOCK,
        LIMELIGHT,
        SLOW_SPEED_MANUAL
    }
    enum SubAssemblyStates {
        INTAKE,
        TRANSFER,
        OUTAKE,
        AUTOOUTAKE,
    }
    enum TransferStates{
        HOME,
        TRANSFER_READY,
        ELSEWHERE
    }



    DrivingStates drivingState = DrivingStates.FULL_SPEED_MANUAL;
    SubAssemblyStates subState = SubAssemblyStates.INTAKE;
    TransferStates transferState = TransferStates.ELSEWHERE;



    @Override
    public void runOpMode() throws InterruptedException {

        fl = hardwareMap.get(DcMotor.class, "FL");
        bl = hardwareMap.get(DcMotor.class, "BL");
        br = hardwareMap.get(DcMotor.class, "BR");
        fr = hardwareMap.get(DcMotor.class, "FR");

        oSlideRight = hardwareMap.get(DcMotor.class, "slidesRight" );
        oSlideLeft = hardwareMap.get(DcMotor.class, "slides_l");

        islide = hardwareMap.get(DcMotor.class, "horSlide" );
        islide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        islide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        oSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        oSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightIntake = hardwareMap.get(Servo.class, "rightIntake" );
        leftIntake = hardwareMap.get(Servo.class, "leftIntake" );



        leftOut = hardwareMap.get(Servo.class, "leftOut");
        OarmLeft = hardwareMap.get(Servo.class, "outArmLeft");
        OarmRight = hardwareMap.get(Servo.class, "outArmRight");
        outRight = hardwareMap.get(Servo.class, "outRight");

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }


        intakeMotorLeft = hardwareMap.get(CRServo.class, "intakeMotorLeft");
        intakeMotorRight = hardwareMap.get(CRServo.class, "intakeMotorRight");
        //leds = hardwareMap.get(RevBlinkinLedDriver.class, "leds");
        //limitSwitch = hardwareMap.get(TouchSensor.class, "limit");






        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();

        new Thread(()->{
            leftIntake.setPosition(0.6104);
            rightIntake.setPosition(0.8425);

            outRight.setPosition(0.8094);
            leftOut.setPosition(0.627);

            OarmRight.setPosition(0.4778);
            OarmLeft.setPosition(0.4778);

            slidesLimiterOutake(1, 16000);
            pause(1);
            slidesLimiterOutake(0, 16000);
        }).start();





        while (opModeIsActive()){
            //There will be no while loops used so everything can run concurrently
            //States will be updated in every loop
            //to deal with the servo delay issues, we send them in a new thread, but check to make sure that thread only executes once
            //chassis state machine
            switch(drivingState){
                case FULL_SPEED_MANUAL:
                    //leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
                    robotCentricPlus(1.3);
                    gp1();
                    break;
                case SLOW_SPEED_MANUAL:
                    //leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE);
                    robotCentricPlus(0.4);
                    gp1();
                    break;
            }
            oSlideLeft.setPower(-gamepad2.right_stick_y);
            oSlideRight.setPower(gamepad2.left_stick_y);

            islide.setPower(gamepad2.right_stick_x);
            //outtakeArm();
            //OarmRight.setPosition(gamepad2.right_stick_y);
            //OarmLeft.setPosition(gamepad2.left_stick_y);



            telemetry.addData("Orightpos: ", oSlideRight.getCurrentPosition());


            //sub state machine
            telemetry.addData("servorightpos: ", OarmRight.getPosition());
            telemetry.addData("servoleftpos: ", OarmLeft.getPosition());

            telemetry.addData("lslides: ", islide.getCurrentPosition());
            telemetry.addData("drivetrain_state: ", drivingState);
            telemetry.addData("subassembly_state: ", subState);
            telemetry.update();
        }


    }
    //START OUTTAKE IN LEVEL POSITION
    public void outakeBox(){
        //specimen
        if(gamepad2.cross){

            outRight.setPosition(1);
            leftOut.setPosition(0.4364);
        }
        //high basket
        else {
            outRight.setPosition(0.3701);
            leftOut.setPosition(1);

        }

        //level, transfer mode
        /*
        if(gamepad2.square){
            outRight.setPosition(0.8094);
            leftOut.setPosition(0.627);
        }
         */
        //627 8094
    }
    public void outtakeArm(){
        //up
        if(gamepad2.right_bumper){
            OarmRight.setPosition(1);
            OarmLeft.setPosition(1);
        }
        //down
        if(gamepad2.left_bumper){
            OarmRight.setPosition(0.4778);
            OarmLeft.setPosition(0.4778);
        }
    }
    public void gp1(){
        //DRIVETRAIN STATE LOGIC
        if (gamepad1.right_trigger > 0.1 && drivingState != DrivingStates.LIMELIGHT){
            drivingState = DrivingStates.SLOW_SPEED_MANUAL;
        }else if(drivingState != DrivingStates.LIMELIGHT){
            drivingState = DrivingStates.FULL_SPEED_MANUAL;
        }
        if(gamepad1.triangle){
            drivingState = DrivingStates.FULL_SPEED_MANUAL;
        }
    }

    public void gp2(){
        //triangle switches intake/outtake
        //ps transfers
        //right/left bumper  intake/outtake

        //cross, toggle intake servo angle
        //left stick y - slides
        //cross - high basket
        if(subState == SubAssemblyStates.OUTAKE && gamepad2.triangle){
            subState = SubAssemblyStates.INTAKE;
            new Thread(()->{
                outRight.setPosition(0.8094);
                leftOut.setPosition(0.627);

                OarmRight.setPosition(0.4778);
                OarmLeft.setPosition(0.4778);

                while(-oSlideRight.getCurrentPosition() > 5000){
                    slidesLimiterOutake(-1, 16000);
                }
                slidesLimiterOutake(0, 16000);
            }).start();

        }
        if(subState == SubAssemblyStates.INTAKE && gamepad2.ps){
            subState = SubAssemblyStates.TRANSFER;
            transfer();
        }
        if(subState == SubAssemblyStates.INTAKE && gamepad2.triangle){
            subState = SubAssemblyStates.OUTAKE;
        }
    }

    public void robotCentricPlus(double speedmult) {
        double x, y, mag, rads, rangle;
        double fieldCentricMultiplier  = 0.8;
        double rotationalMult = -0.8;
        x = -fieldCentricMultiplier * (gamepad1.left_stick_x) * speedmult;
        y = -fieldCentricMultiplier * (gamepad1.left_stick_y) * speedmult;
        double rotational = gamepad1.right_stick_x * rotationalMult * speedmult;
        mag = Math.pow(x, 2) + Math.pow(y, 2);
        mag = Math.sqrt(mag);
        rads = Math.atan2(y, x);
        rads = (rads >= 0 && rads < Math.toRadians(270)) ? (-1 * rads) + Math.toRadians(90) : (-1 * rads) + Math.toRadians(450);
        rads = (rads < 0) ? Math.toRadians(360) + rads : rads;
        double turn = rads;
        double equationone = (Math.sin(turn + (Math.PI / 4)) * mag);
        double equationtwo = -(Math.sin(turn - (Math.PI / 4)) * mag);
        fr.setPower((equationone + rotational));
        bl.setPower((equationone - rotational));
        br.setPower((equationtwo + rotational));
        fl.setPower((equationtwo - rotational));
    }
    public void intakeServos(){
        if(gamepad2.right_bumper) {
            intakeMotorRight.setPower(0.5);
            intakeMotorLeft.setPower(-0.5);
        } else if(gamepad2.left_bumper){
            intakeMotorRight.setPower(-0.5);
            intakeMotorLeft.setPower(0.5);
        }else{
            intakeMotorRight.setPower(0);
            intakeMotorLeft.setPower(0);
        }

        if(gamepad2.cross) {
            if (rightIntake.getPosition() != 0.8425) {
                //drop down
                leftIntake.setPosition(0.52);
                rightIntake.setPosition(0.7521);
            } else {
                //level
                leftIntake.setPosition(0.6104);
                rightIntake.setPosition(0.8425);
            }
        }
    }
    public void slidesLimiterIntake(double power, DcMotor lslides, int upperBound){
        if((lslides.getCurrentPosition() > upperBound && power > 0) || (lslides.getCurrentPosition() < 0 && power < 0)) {
            lslides.setPower(0);
        } else{
            lslides.setPower(power);
        }
    }
    public void slidesLimiterOutake(double power, int upperBound){
        if((-oSlideRight.getCurrentPosition() > upperBound && power > 0) || -oSlideRight.getCurrentPosition() < 0 && power < 0) {
            oSlideRight.setPower(0);
            oSlideLeft.setPower(0);

        } else{
            oSlideLeft.setPower(power);
            oSlideRight.setPower(-power);

        }
    }


    public void pSlideController(DcMotor lslides, int reference, double feedforward){
        double error = reference - lslides.getCurrentPosition();
        double output =  (error/1000) + feedforward;
        lslides.setPower(output);
    }

    public boolean toleranceChecker(double num, double ref, double tolerance){
        return Math.abs((Math.abs(ref) + tolerance) - Math.abs(num)) <= tolerance;
    }
    public void transfer() {
        new Thread(()->{
            leftIntake.setPosition(0.6104);
            rightIntake.setPosition(0.8425);

            outRight.setPosition(0.8094);
            leftOut.setPosition(0.627);

            OarmRight.setPosition(0.4778);
            OarmLeft.setPosition(0.4778);

            while(-oSlideRight.getCurrentPosition() < 4000){
                slidesLimiterOutake(1, 16000);
            }
            slidesLimiterOutake(0, 16000);

            while(islide.getCurrentPosition() > 20){
                slidesLimiterIntake(-1, islide, 1300);
            }
            slidesLimiterIntake(0, islide, 1300);

            while(-oSlideRight.getCurrentPosition() > 20){
                slidesLimiterOutake(-1, 16000);
            }
            slidesLimiterOutake(0, 16000);

            leftIntake.setPosition(0.7679);
            rightIntake.setPosition(1);
            pause(0.8);
            leftIntake.setPosition(0.6104);
            rightIntake.setPosition(0.8425);

            while(-oSlideRight.getCurrentPosition() < 4000){
                slidesLimiterOutake(1, 16000);
            }
            slidesLimiterOutake(0, 16000);
            OarmRight.setPosition(1);
            OarmLeft.setPosition(1);
            outRight.setPosition(0.3701);
            leftOut.setPosition(1);
            subState = SubAssemblyStates.OUTAKE;
        }).start();
        //first check that subs are homed
        //Tilt the servos to transfer the block
        //if the outtake is primed, bring the slides back and level the intake
    }
    public void pause(double seconds){
        ElapsedTime timer = new ElapsedTime();
        while (timer.seconds() < seconds){
            continue;
        }
    }




}
