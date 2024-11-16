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

@TeleOp(name="TeleOp")
public class ITDTeleOp extends LinearOpMode {
    DcMotor fl,fr,br,bl, oSlideRight, oSlideLeft, islide;

    Servo outRight, leftOut, rightIntake, leftIntake, OarmLeft, OarmRight;


    CRServo intakeMotorLeft, intakeMotorRight;
    //RevBlinkinLedDriver leds;
    TouchSensor limitSwitch;
    //34.1632
    enum DrivingStates {
        FULL_SPEED_MANUAL,
        LIMELIGHT,
        SLOW_SPEED_MANUAL
    }
    enum SubAssemblyStates {
        INTAKE,
        TRANSFER,
        OUTAKE,
        AUTOOUTAKE,
    }

    DrivingStates drivingState = DrivingStates.FULL_SPEED_MANUAL;
    SubAssemblyStates subState = SubAssemblyStates.INTAKE;


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
        islide.setDirection(DcMotorSimple.Direction.REVERSE);

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
            setIntakeLevel();
            setOUTTAKE_ARM_DOWN();
            setOUTTAKE_LEVEL();
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
            switch(subState){
                case INTAKE:
                    slidesLimiterIntake(-gamepad2.left_stick_y, islide, 1300);
                    intakeServos();
                    gp2();
                    break;
                case OUTAKE:
                    slidesLimiterOutake(-gamepad2.left_stick_y, 18000);
                    outakeBox();
                    gp2();
                    break;
            }




            telemetry.addData("vertical slides: ", oSlideRight.getCurrentPosition());
            telemetry.addData("intake slides: ", islide.getCurrentPosition());
            telemetry.addData("drivetrain_state: ", drivingState);
            telemetry.addData("subassembly_state: ", subState);
            telemetry.update();
        }


    }
    //START OUTTAKE IN LEVEL POSITION
    public void outakeBox(){
        //high basket
        if(gamepad2.cross){
            setOUTTAKE_DROP();
        }
        //specimen
        else {
            setOUTTAKE_SPECIMEN();
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
        //change to intake mode
        if(subState == SubAssemblyStates.OUTAKE && gamepad2.triangle){
            setOUTTAKE_LEVEL();
            setOUTTAKE_ARM_DOWN();
            while(-oSlideRight.getCurrentPosition() > 2000){
                slidesLimiterOutake(-1, 16000);
            }
            slidesLimiterOutake(0, 16000);

            subState = SubAssemblyStates.INTAKE;
            sleep(200);
        }
        //transfer
        if(subState == SubAssemblyStates.INTAKE && gamepad2.ps){
            subState = SubAssemblyStates.TRANSFER;
            transfer();
        }
        //change to outtake mode
        if(subState == SubAssemblyStates.INTAKE && gamepad2.triangle){
            setOUTTAKE_ARM_UP();
            setOUTTAKE_SPECIMEN();
            subState = SubAssemblyStates.OUTAKE;
            sleep(200);
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
            intakeMotorRight.setPower(0.7);
            intakeMotorLeft.setPower(-0.7);
        } else if(gamepad2.left_bumper){
            intakeMotorRight.setPower(-0.7);
            intakeMotorLeft.setPower(0.7);
        }else{
            intakeMotorRight.setPower(0);
            intakeMotorLeft.setPower(0);
        }

        if(gamepad2.cross) {
            if (rightIntake.getPosition() != 0.7521) {
                //drop down
                setIntakeDropDown();
                sleep(200);
            } else {
                //level
                setIntakeLevel();
                sleep(200);
            }
        }
    }
    public void slidesLimiterIntake(double power, DcMotor lslides, int upperBound){
        if((-lslides.getCurrentPosition() > upperBound && power > 0) || (-lslides.getCurrentPosition() < 0 && power < 0)) {
            lslides.setPower(0);
        } else{
            //hover the slides at like 300 ticks
            slidesLimiterOutake(0.5,300);
            lslides.setPower(-power);
        }
    }
    public void slidesLimiterOutake(double power, int upperBound) {
        if (!gamepad2.dpad_down){
            if ((-oSlideRight.getCurrentPosition() > upperBound && power > 0) || -oSlideRight.getCurrentPosition() < 0 && power < 0) {
                oSlideRight.setPower(0);
                oSlideLeft.setPower(0);

            } else {
                oSlideLeft.setPower(power);
                oSlideRight.setPower(-power);

            }
        }else{
            oSlideLeft.setPower(-0.5);
        }
    }

    public void transfer() {
        //TO DO: THREAD SUBASSEMBLIES BUT MAKE SURE ONLY ONE THREAD CAN BE OPENED
        //ADD ASYNC TIMEOUT Feature for some buttons
        //ADD Time based killswitch for slides
        setIntakeLevel();
        setOUTTAKE_LEVEL();
        setOUTTAKE_ARM_DOWN();

        while((-islide.getCurrentPosition()) > 20){
            slidesLimiterIntake(-0.5, islide, 1300);
        }
        slidesLimiterIntake(0, islide, 1300);

        while(-oSlideRight.getCurrentPosition() > 20){
            slidesLimiterOutake(-1, 16000);
        }
        slidesLimiterOutake(0, 16000);


        setINTAKE_TRANSFER();
        pause(1.5);
        setOUTTAKE_TRANSFER();
        setIntakeLevel();


        while(-oSlideRight.getCurrentPosition() < 2000){
            slidesLimiterOutake(1, 16000);
        }
        slidesLimiterOutake(0, 16000);



        OarmRight.setPosition(0.7);
        OarmLeft.setPosition(0.7);

        setOUTTAKE_SPECIMEN();
        setOUTTAKE_ARM_UP();

        subState = SubAssemblyStates.OUTAKE;
        //first check that subs are homed
        //Tilt the servos to transfer the block
        //if the outtake is primed, bring the slides back and level the intake
    }
    //SERVO_POSES
    public void setOUTTAKE_ARM_DOWN(){
        OarmRight.setPosition(0.4778);
        OarmLeft.setPosition(0.4778);
    }
    public void setOUTTAKE_ARM_UP(){
        OarmRight.setPosition(1);
        OarmLeft.setPosition(1);
    }

    public void setIntakeDropDown(){
        leftIntake.setPosition(0.52);
        rightIntake.setPosition(0.7521);
    }
    public void setIntakeLevel(){
        leftIntake.setPosition(0.6104);
        rightIntake.setPosition(0.8425);
    }
    public void setINTAKE_TRANSFER(){
        leftIntake.setPosition(0.6979);
        rightIntake.setPosition(0.93);
    }

    public void setOUTTAKE_LEVEL(){
        outRight.setPosition(0.8094);
        leftOut.setPosition(0.627);
    }
    public void setOUTTAKE_TRANSFER(){
        outRight.setPosition(0.9094);
        leftOut.setPosition(0.527);
    }
    public void setOUTTAKE_SPECIMEN(){
        outRight.setPosition(0.3701);
        leftOut.setPosition(1);
    }
    public void setOUTTAKE_DROP(){
        outRight.setPosition(1);
        leftOut.setPosition(0.4364);
    }

    public void pause(double seconds){
        ElapsedTime pauseTimer = new ElapsedTime();
        pauseTimer.reset();
        while(pauseTimer.seconds() < seconds){
            continue;
        }
    }

}
