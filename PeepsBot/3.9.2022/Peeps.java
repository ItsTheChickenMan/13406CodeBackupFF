package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import AutoTools.EzrasLaw;

@TeleOp

public class Peeps extends EzrasLaw {
    public DcMotorEx swivelMotor;
    public DcMotorEx armMotor;
    public DcMotor intake;
    
    public ElapsedTime timer;
    
    public double armMotorTPR = 1425.1;
    
    public void initOpMode(){
        swivelMotor = hardwareMap.get(DcMotorEx.class, "swivelMotor");
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        intake = hardwareMap.get(DcMotor.class, "test");
        
        timer = new ElapsedTime();
    }
    
    // todo: write your code here   q11111111111111111111112w
    @Override
    public void runOpMode(){
        this.initOpMode();
        
        swivelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        waitForStart();
        
        intake.setPower(-1);
        
        /*swivelMotor.setTargetPosition((int)position);
        
        swivelMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        swivelMotor.setPower(1);
        */
        /*armMotor.setTargetPosition(75);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setVelocity(this.armMotorTPR*6);
        
        //while(armMotor.isBusy() && opModeIsActive());
        
        swivelMotor.setTargetPosition(-1000);
        
        swivelMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        swivelMotor.setPower(0.55);
        
        wait(2.0);
        
        swivelMotor.setTargetPosition(0);
        
        while(swivelMotor.isBusy() && opModeIsActive());
        
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        */
        
        while(opModeIsActive()){
            telemetry.addData("currentPosition", swivelMotor.getCurrentPosition());
            telemetry.update();
        };
    }
    
    public void wait(double time){
        double startTime = timer.seconds();
        
        while(timer.seconds() < startTime+time && opModeIsActive());
    }
}