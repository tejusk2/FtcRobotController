package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
public class Deadwheel {
    DcMotor deadwheel;
    public Deadwheel(DcMotor motor){
        deadwheel = motor;
    }
    public int getCurrentPosition(){return deadwheel.getCurrentPosition();}
    public void reset(){
        deadwheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        deadwheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}

//one that resets to 0 and runs without encoders
