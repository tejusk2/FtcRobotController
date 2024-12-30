package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.RobotDeprecated;

import java.util.List;

@Autonomous(name="observation")
public class ObservationSide extends LinearOpMode {
    DcMotor fl,fr,br,bl, enc_left, leftSlides, rightSlides, islides;
    ServoImplEx outRight, leftOut, rightIntake, leftIntake, OarmLeft, OarmRight;


    CRServo intakeMotorLeft, intakeMotorRight;
    RevBlinkinLedDriver leds;
    TouchSensor left, right;    //34.1632

    @Override
    public void runOpMode() throws InterruptedException {
        fl = hardwareMap.get(DcMotor.class, "FL");
        bl = hardwareMap.get(DcMotor.class, "BL");
        br = hardwareMap.get(DcMotor.class, "BR");
        fr = hardwareMap.get(DcMotor.class, "FR");

        leftSlides = hardwareMap.get(DcMotor.class, "slides_l");

        enc_left = hardwareMap.get(DcMotor.class, "enc_left");

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        rightSlides = hardwareMap.get(DcMotor.class, "slidesRight" );
        leftSlides = hardwareMap.get(DcMotor.class, "slides_l");

        islides = hardwareMap.get(DcMotor.class, "horSlide" );
        islides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        islides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //
        islides.setDirection(DcMotorSimple.Direction.REVERSE);

        rightSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightIntake = hardwareMap.get(ServoImplEx.class, "rightIntake" );
        leftIntake = hardwareMap.get(ServoImplEx.class, "leftIntake" );


        left = hardwareMap.get(TouchSensor.class, "left");
        right = hardwareMap.get(TouchSensor.class, "right");

        leftOut = hardwareMap.get(ServoImplEx.class, "leftOut");
        OarmLeft = hardwareMap.get(ServoImplEx.class, "outArmLeft");
        OarmRight = hardwareMap.get(ServoImplEx.class, "outArmRight");
        outRight = hardwareMap.get(ServoImplEx.class, "outRight");


        intakeMotorLeft = hardwareMap.get(CRServo.class, "intakeMotorLeft");
        intakeMotorRight = hardwareMap.get(CRServo.class, "intakeMotorRight");

        leds = hardwareMap.get(RevBlinkinLedDriver.class, "LED");
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        enc_left.setDirection(DcMotorSimple.Direction.REVERSE);
        leftSlides.setDirection(DcMotorSimple.Direction.REVERSE);


        RobotDeprecated robot = new RobotDeprecated(fr, fl, br, bl, enc_left, leftSlides, islides, rightSlides, leds,
                OarmLeft, OarmRight, outRight, leftOut, rightIntake, leftIntake, left, right, intakeMotorLeft, intakeMotorRight);
        encodeReset();
        OarmRight.setPwmEnable();
        OarmLeft.setPwmEnable();
        leftOut.setPwmEnable();
        outRight.setPwmEnable();
        while(opModeInInit()){
            OarmRight.setPosition(0.4);
            OarmLeft.setPosition(0.4);
            leftOut.setPosition(0);
            outRight.setPosition(0);
        }
        waitForStart();

        OarmRight.setPosition(0.85);
        OarmLeft.setPosition(0.85);
        leftOut.setPosition(1);
        outRight.setPosition(1);
        /*
        for (double i = 0; i <= 100; i += 0.5) {
            leftOut.setPosition(0.0 + (i * (1 / 100)));
            outRight.setPosition(0.0 + (i * (1 / 100)));
            OarmRight.setPosition(0.4 + (i * (0.45 / 100)));
            OarmLeft.setPosition(0.4 + (i * (0.45 / 100)));
        }
         */

        //forward
        //positive x direction is left
        //outake side forward is negative y direction

        robot.motionProfile(13, -25);
        sleep(200);
        robot.timedCreep(0.6, -0.3);
        robot.specimenDropoffPosition(310);
        sleep(200);
        while((left.isPressed() && right.isPressed())){
            robot.slidesLimiterOutake(-.8, 2000, -100);
        }
        if((!left.isPressed() || !right.isPressed()) && -rightSlides.getCurrentPosition() > 250){
            while(-rightSlides.getCurrentPosition() > 10){
                robot.slides(-0.9);
            }
        }
        robot.slidesLimiterOutake(0, 2000, 0);
        sleep(200);
        OarmRight.setPosition(0.4);
        OarmLeft.setPosition(0.4);
        leftOut.setPosition(0);
        outRight.setPosition(0);

        robot.motionProfile(0, 6);
        robot.motionProfile(-30, 0);
        robot.motionProfile(0, -27);
        robot.motionProfile(-15, 0);
        robot.motionProfile(0, 43);
        robot.motionProfile(0, -43);
        robot.motionProfile(-15, 0);
        robot.motionProfile(0, 43);
        robot.motionProfile(0, -43);
        robot.motionProfile(-15, 0);
        robot.motionProfile(0, 43);










    }
    public void encodeReset(){
        enc_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

}
