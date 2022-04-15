/*
Copyright 2022 FIRST Tech Challenge Team FTC_13406

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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

public class ArmAlignment extends LinearOpMode {
    private Blinker control_Hub;
    private Blinker expansion_Hub_2;
    private HardwareDevice webcam_1;
    private DcMotor armMotor;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor carousel;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private Gyroscope imu;
    private CRServo tapeBlue;
    private CRServo tapeRed;
    private ElapsedTime globalTime;

    @Override
    public void runOpMode() {
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        globalTime = new ElapsedTime();
        
        // Wait for the game to start (driver presses PLAY)
        double timeDelay = 0.0;
        
        while(!isStarted()){
            // inform user of settings
            telemetry.addLine("Move left joystick to change time delay setting");
            telemetry.addData("Time Delay", timeDelay);
            
            telemetry.update();
            
            timeDelay += -gamepad1.left_stick_y/5000;
            
            timeDelay = timeDelay < 0 ? 0 : timeDelay;
        }
        
        telemetry.addLine("Move arm to initial position and press A to initialize");
        telemetry.update();
        
        while(!gamepad1.a && opModeIsActive());
        
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        String[] prompts = {"Move arm to grabbing position", "Move arm to bottom of hub", "Move arm to middle of hub", "Move arm to top of hub", "Move hub to capping position"};
        double[] positions = new double[5];
        
        for(int i = 0; i < prompts.length; i++){
            // wait for user to unpress a so that it doesn't fly through the instructions
            while(gamepad1.a && opModeIsActive());
            
            telemetry.addLine(prompts[i]);
            telemetry.addLine("Press A when ready to read data");
            
            telemetry.update();
            
            while(!gamepad1.a && opModeIsActive());
            
            if(timeDelay > 0){
                double startTime = this.globalTime.seconds();
                
                while(this.globalTime.seconds() < startTime+timeDelay && opModeIsActive()){
                    double remaining = timeDelay - (this.globalTime.seconds() - startTime);
                    telemetry.addLine("Reading in " + Math.round(remaining) + " seconds...");
                    telemetry.update();
                }
            }
            
            // read val
            positions[i] = (double)armMotor.getCurrentPosition();
            
            telemetry.addData("Current val is", positions[i]);
            telemetry.addLine("Press A to proceed with this val or B to change");
            
            telemetry.update();
            
            while(!gamepad1.a && !gamepad1.b && opModeIsActive());
            
            if(gamepad1.b){
                i--;
            }
        }
        
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Grabbing position", positions[0]);
            telemetry.addData("Bottom", positions[1]);
            telemetry.addData("Middle", positions[2]);
            telemetry.addData("Top", positions[3]);
            telemetry.addData("Cap", positions[4]);
            telemetry.update();
        }
    }
}

/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.CRServo;


public class ArmAlignment {
    private Blinker control_Hub;
    private Blinker expansion_Hub_2;
    private HardwareDevice webcam_1;
    private DcMotor armMotor;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor carousel;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private Gyroscope imu;
    private CRServo tapeBlue;
    private CRServo tapeRed;

    // todo: write your code here
}*/