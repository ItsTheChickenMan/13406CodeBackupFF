/*
Copyright 2022 FIRST Tech Challenge Team FTC

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import AutoTools.EzrasLaw;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@Autonomous

public class PhoenixAutoBlueWarehouse extends EzrasLaw {
    // SETTINGS
    
    // how long to wait for a pipe recognition
    private double pipeWaitTime = 1.0;
    
    // how long to spin carousel for duck
    private double duckWait = 2.2;
    
    // zoom for tfod
    private double tfodZoom = 1.1;
    
    // each barcode's corresponding hub level
    // indices: 0 = left 1 = mid 2 = right
    // values: 0 = bottom 1 = mid 2 = top
    private int[] hubOrder = {0, 1, 2};
    
    // MISC GLOBALS
    
    // what hubl evel to place pre-loaded block onto
    // 0 = bottom 1 = mid 2 = top
    private int hubLevel;
    
    public void initAuto(){
        initVals();
        
        this.unclamp();
        
        initVuforia();
        initTfod();
        
        if (tfod != null) {
            tfod.activate();
            
            tfod.setZoom(tfodZoom, 16.0/9.0);
        }
        
        initArm();
        
        setupIMU();
        
        this.clamp();
        
        this.frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        this.frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        this.backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        this.backRight.setDirection(DcMotorSimple.Direction.REVERSE);
    
        this.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    
        // bottom should be out of frame, default to bottom
        this.hubLevel = 0;
    }

    @Override
    public void runOpMode() {
        initAuto();
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
        globalTime = new ElapsedTime();
        
        // get position of pipe in frame
        float pipeDetection = waitForPipeRecognition(pipeWaitTime);
        
        // middle
        if(pipeDetection < 200 && pipeDetection > 0){
            this.hubLevel = 1;
        } else if(pipeDetection > 200){ // right
            this.hubLevel = 2;
        } else { // right
            // this should be default anyways, but set it just for fun
            this.hubLevel = 0;
        }
        
        // update step
        telemetry.addData("Got detection, this.hubLevel", this.hubLevel == 0 ? "bottom" : this.hubLevel == 1 ? "middle" : "top");
        telemetry.addLine("Moving to hub");
        telemetry.update();
        
        // move to hub
        this.strafeDistance(15.5, 0.5, true);
        
        // pivot to have arm face hub
        this.pivotAngle(-180.0, 0.8);
        
        // move arm to level
        // top 22
        // middle 25
        // bottom 28
        int offset = 25;
        
        if(this.hubLevel == 1){
            offset = 28;    
        } else if(this.hubLevel == 0){
            offset = 29;
        }
        
        setArmLevel(this.hubLevel+1, offset, 0.2);
        
        telemetry.addLine("waiting for arm");
        telemetry.update();
        
        // make sure arm is good
        while(armMotor.isBusy() && opModeIsActive());
        
        // keep going to hub
        telemetry.addLine("going to hub");
        telemetry.update();
        
        // move forward to line arm up
        this.goForwardDistance(19, 0.5, true);
        
        // deposit
        telemetry.addLine("depositing");
        telemetry.update();
        
        this.unclamp();
        
        // wait a sec
        wait(0.6);
        
        telemetry.addLine("deposited");
        telemetry.addLine("parking...");
        telemetry.update();
        
        // do everything we just did but in reverse
        this.goForwardDistance(-19, 0.5, true);
        
        telemetry.addLine("resetting arm");
        telemetry.update();
        
        // move arm back
        this.setArmPosition(-200, 0.4);
        
        // pivot 180
        this.pivotAngle(-180.0, 0.8);
        
        // kill arm
        this.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        // strafe back against wall
        this.strafeDistance(-18, 0.5, true);
        
        // move into storage
        this.goForwardDistance(40, 0.5, true);
        
        // strafe back to prevent arm damage
        this.strafeDistance(24, 0.5, true);

        // move to hub
        /*this.goForwardDistance(4, 0.5, true);
        
        double totalDistance= 64;
        double partialDistance = 30;
        
        this.strafeDistance(partialDistance, 0.5, true);
        
        // move arm to position
        telemetry.addLine("Moving arm to position");
        telemetry.addLine("Moving to hub");
        telemetry.update();
        
        // move arm to level
        // top 22
        // middle 25
        // bottom 28
        int offset = 25;
        
        if(this.hubLevel == 1){
            offset = 28;    
        } else if(this.hubLevel == 0){
            offset = 29;
        }
        
        setArmLevel(this.hubLevel+1, offset, 0.2);
        
        // move rest of way
        this.strafeDistance(totalDistance-partialDistance, 0.5, true);
        
        telemetry.addLine("waiting for arm");
        telemetry.update();
        
        // make sure arm is good
        while(armMotor.isBusy() && opModeIsActive());
        
        // move rest of way to hub
        this.goForwardDistance(-22, 0.4, true);
        
        telemetry.addLine("depositing");
        telemetry.update();
        
        // drop block
        this.unclamp();
        
        // wait a sec
        wait(0.6);
        
        telemetry.addLine("going back");
        telemetry.update();
        
        // move back
        this.goForwardDistance(22, 0.5, true);
        
        // move arm back
        this.setArmPosition(-200, 0.4);
        
        // strafe back into wall
        this.strafeDistance(-partialDistance, 0.5, true);
        
        // stop arm
        this.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        // move rest of way into wall
        this.strafeDistance(-(totalDistance-partialDistance) - 4, 0.5, true);
        
        // move into field
        telemetry.addLine("to da field");
        telemetry.update();
        
        this.goForwardDistance(40, 0.8, true);*/
        
        // run until the end of the match (driver presses STOP)
        String message = getRandomFinalMessage();
        while (opModeIsActive()) {
            telemetry.addLine(message);
            telemetry.update();
        }
    }
}
