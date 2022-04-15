package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp

public class OLDWinningOpModeRed extends OLDWinningOpModeBlue {
  // the only difference is swapped tapes and carousel direction, so swap blueTape
  @Override
  public void setupTape(){
    this.blueTape = 0;
    this.tape = this.tapeRed;
  }
}
