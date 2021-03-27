// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.newvision;

import java.util.ArrayList;

import frc.robot.RobotContainer;
import frc.robot.vision.ColorBlock;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;

/** Add your docs here. */
public class PixyHelper {
    
	private static ArrayList<Block> blocks = new ArrayList<>();
	private static ArrayList<ColorBlock> returnBlocks = new ArrayList<>();
   
    public static ArrayList<ColorBlock> getBlocks() {
        Pixy2 pixy = RobotContainer.pixy;
        pixy.getCCC().getBlocks(false, 255, 3);

        blocks = pixy.getCCC().getBlockCache();

		returnBlocks.clear();

		for (int i = 0; i < blocks.size(); i++) {
			returnBlocks.add(new ColorBlock(blocks.get(i)));
		}

		

		return returnBlocks;
    }
    
    public static String getBlocksString() {
        var blocks = getBlocks();
        var rtrn = "";
        for(int i = 0; i < blocks.size(); i++) {
            rtrn += "[" + blocks.get(i).getX() + ",  ";
            rtrn += blocks.get(i).getY() + ",  ";
            rtrn += blocks.get(i).getWidth() + ",  ";
            rtrn += blocks.get(i).getLength() + "]";

        }

        return rtrn;
    }
}
