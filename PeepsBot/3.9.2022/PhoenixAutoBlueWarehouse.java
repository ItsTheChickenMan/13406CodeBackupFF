package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import AutoTools.EzrasLaw;

@Autonomous

public class PhoenixAutoBlueWarehouse extends EzrasLaw {
    public void initAuto(){
        this.initVals();
        
        this.setupIMU();
        
        this.initArm();
        
        this.initializeSwivel();
        
        this.globalTime = new ElapsedTime();
        
        // setup motor directions
        this.frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        this.frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        this.backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        this.backRight.setDirection(DcMotorSimple.Direction.REVERSE);
    
        this.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        initVuforia();
        initTfod();
        
        if (tfod != null) {
            tfod.activate();
            
            tfod.setZoom(1.1, 16.0/9.0);
        }
    }
    
    @Override
    public void runOpMode(){
        this.initAuto();
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        waitForStart();
        
        // immediately clamp
        this.clamp();
        
        // lift arm
        this.setArmRotation(this.armWaitPosition);
    
        // get position of pipe in frame
        float pipeDetection = waitForPipeRecognition(0.5);
        
        int hubLevel = 2; // 0 = bottom, 1 = middle, 2 = top
        
        // middle
        if(pipeDetection < 200 && pipeDetection > 0){
            hubLevel = 1;
        } else if(pipeDetection > 200){ // right
            hubLevel = 2;
        } else { // right
            // this should be default anyways, but set it just for fun
            hubLevel = 0;
        }
        
        double armPosition = this.armPositions[hubLevel+1];
        
        telemetry.addData("hubLevel", hubLevel + "");
        telemetry.update();
        
        // start moving
        this.compoundMove(4.0, 0.0, 26.0, 0.0);
        
        while(!this.isDone() && opModeIsActive());
        
        this.compoundMove(24.0, -24.0, 26.0, 0.0);
        
        // wait until arm is at position, then move swivel
        while(!this.isDone() && opModeIsActive()){
            if(!this.armMotor.isBusy() && !this.swivelMotor.isBusy()){
                this.setSwivelAngle(60.0);
            }
            
            if(this.getSwivelRotation() > this.swivelWaitPosition){
                this.setArmRotation(armPosition);
            }
        }
        
        while(this.swivelMotor.isBusy() && opModeIsActive());
        
        this.unclamp();
        
        this.compoundMoveStraight(-24.0, 26.0, 26.0, 0.0);
        
        this.waitForDrive();
        
        this.setArmRotation(this.armWaitPosition);
        
        this.compoundMove(-32, 0.0, 26.0, 0.0);
        
        this.waitForDrive();
        
        while(this.swivelMotor.isBusy() && opModeIsActive());
        
        this.setSwivelAngle(this.swivelRestingPosition);
        
        this.compoundMove(0.0, -26.0, 26.0, 0.0);
        
        this.waitForDrive();
    }
    
    public void waitForDrive(){
        while(!this.isDone() && opModeIsActive());
    }
}