package org.firstinspires.ftc.teamcode.Motors;

import com.qualcomm.robotcore.hardware.configuration.DistributorInfo;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.ExpansionHubPIDFPositionParams;
import com.qualcomm.robotcore.hardware.configuration.annotations.ExpansionHubPIDFVelocityParams;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

@MotorType(ticksPerRev = 537.6, gearing = 19.2, maxRPM = 340, orientation = Rotation.CCW)
// unsure about orientation for this one
@DeviceProperties(xmlTag = "NeveRestOrbital20", name = "NeveRest Orbital 20", builtIn = true)
@DistributorInfo(distributor = "AndyMark", model = "am-3637", url = "https://www.andymark.com/products/neverest-orbital-20-gearmotor")
@ExpansionHubPIDFVelocityParams(P = 2.0, I = 0.5, F = 11.0)
//currently same as the old 20 gearmotor we used, but it may be closer to the pidf values of the gobilda 19.2:1
@ExpansionHubPIDFPositionParams(P = 5.0)
public interface NeveRestOrbital20 {
}