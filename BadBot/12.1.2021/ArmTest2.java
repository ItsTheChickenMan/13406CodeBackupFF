/*
Copyright 2021 FIRST Tech Challenge Team FTC_13406

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
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

public class ArmTest2 extends LinearOpMode {
    DcMotorEx armMotor;
    Servo clawServoLeft;
    Servo clawServoRight;
    
    @Override
    public void runOpMode() {
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        clawServoLeft = hardwareMap.get(Servo.class, "clawServoLeft");
        clawServoRight = hardwareMap.get(Servo.class, "clawServoRight");
        
        // Reset the encoder during initialization
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        //clawServoLeft.getController().pwmEnable();
        //clawServoRight.getController().pwmEnable();
        
        clawServoRight.scaleRange(0.0, 0.885); // 
        
        waitForStart();
        
        int armPosition = -300;
        int lastArmPosition = armPosition;
        
        // Set the motor's target position to 300 ticks
        armMotor.setTargetPosition(armPosition);
        armMotor.setPower(0.75);
        
        // Switch to RUN_TO_POSITION mode
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        boolean active;
        boolean clamped;
        
        // While the Op Mode is running, show the motor's status via telemetry
        while (opModeIsActive()) {
            telemetry.addData("velocity", armMotor.getVelocity());
            telemetry.addData("position", armMotor.getCurrentPosition());
            telemetry.addData("is at target", !armMotor.isBusy());
            telemetry.update();
            
            double movement_y = Range.clip(-gamepad1.left_stick_y, -1.0, 1.0);
            
            if(movement_y != 0){
                armPosition -= movement_y;
                
                armMotor.setTargetPosition(armPosition);
                armMotor.setPower(0.75);
                
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            
            if(!gamepad1.a){
                clawServoLeft.setPosition(0.0);
                clawServoRight.setPosition(1.0);
            } else {
                clawServoLeft.setPosition(0.6);
                clawServoRight.setPosition(0.55);
            }
        }
    }
}
