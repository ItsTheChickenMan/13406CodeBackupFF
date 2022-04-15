/*
Copyright 2022 FIRST Tech Challenge Team 13406

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
import com.qualcomm.robotcore.hardware.DistanceSensor;
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
@TeleOp

public class WinningOpModeBlue extends EzrasLaw {
    // dummy distance sensor
    public DistanceSensor blockSensor; // sensor for block at end of intake
    
    // SETTINGS //
    
    // drive mode
    public int driveMode = 0; // 0 - automatic 1 - manual
    
    // claw state
    public boolean clamped = false; // false - unclamped, true - clamped
    
    // swivel
    public double swivelRotation = 0; // current rotation of the swivel
    public double swivelRestingPosition = 0; // resting position of swivel, as an angle
    public double swivelOutPosition = 60; // out position of swivel, as an angle
    
    // intake
    public double intakePower; // 0 - off, -1 - backwards, 1 - forwards
    
    
    // STATES //
    public boolean blockInSlot = false; // if block is in bucket slot
    
    public boolean swivelAtRest = false; // if the swivel is in its resting position
    
    // initialize opmode
    public void initOpMode(){
        this.initVals();
        
        this.setupIMU();
        
        this.initArm();
    }

    @Override
    public void runOpMode() {
        this.initOpMode();
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();
                
            
        }
    }
    
    // drive modes
    
    /**
     * @brief Update each state to its appropriate value
    */
    public void updateStates(){
        // update block in slot check
        
        // swivel at rest check
    }
    
    /**
     * @brief Check each current setting and react accordingly 
    */
    public void checkSettings(){
        
    }
    
    /**
     * @brief Check gamepad b and handle swivel/arm/claw action states
     * 
    */
    public void automaticClawHandling(){
        // general logic:
        // if swivel is at rest and intaking, check if there is a block in the slot.  if there is, update clamp state, reverse the intake state, and set the swivel rotation to whatever the right position is
            // during this time clamp control should be disabled
        // if swivel is out and clamped state is false, swing back to default position
        
        // check if swivel is at rest and we're intaking
        if(this.swivelAtRest && this.intakePower == 1){
            // block in slot check
            if(this.blockInSlot){
                // update clamp setting
                this.clamped = true;
                
                // reverse intake power
                this.intakePower = -1;
                
                // set swivel rotation
                this.swivelRotation = this.swivelOutPosition; // TODO: actual rotation!
            }
        // check if swivel is out and we unclamped
        } else if(!this.swivelAtRest && !this.clamped){
            // return swivel rotation to resting position
            this.swivelRotation = this.swivelRestingPosition;
        }
        
        // controls
        
    }
    
    // dummy functions
    
}
