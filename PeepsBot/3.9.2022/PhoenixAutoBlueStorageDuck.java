package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous

public class PhoenixAutoBlueStorageDuck extends PhoenixAutoBlueDuck {
    @Override
    public void initOtherVals(){
        this.parkInWarehouse = true;
    }
}