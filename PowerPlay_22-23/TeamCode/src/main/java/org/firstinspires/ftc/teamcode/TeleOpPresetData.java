package org.firstinspires.ftc.teamcode;

public class TeleOpPresetData {
    public double posX, posY, posA;
    public double hPickupLen, hPickupPan, hPickupTilt1, hPickupTilt2;

    // always has 4 pole data(0 ~ 3), but one of them not available
    public final int polePosNum = 4;
    public TeleVSlidePresetPos[] teleVSlidePresetPos = new TeleVSlidePresetPos[polePosNum];
    public boolean hSlideMove = false;
    public int defaultPolePos = 0;


    public TeleOpPresetData(){
        //do nothing
    }








}
