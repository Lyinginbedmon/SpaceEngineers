// == Truckleweed OS ==
const string version = "1.6";

// Block references
List<IMyGyro> gyros = new List<IMyGyro>();
List<IMyThrust> cruiseThrusters = new List<IMyThrust>();
List<IMyThrust> reverseThrusters = new List<IMyThrust>();
List<IMyLandingGear> parkingLegs = new List<IMyLandingGear>();
List<IMyBatteryBlock> batteries = new List<IMyBatteryBlock>();
List<IMyPowerProducer> engines = new List<IMyPowerProducer>();
IMyLightingBlock cruise;
IMyTextPanel screen;
IMyShipController cockpit;

// Block name references
string cruiseIndicator = "Cruise Control Indicator";

// Gyro control values
Vector3D reference;
Vector3D target;
double gravity;

int cruisingSpeed = 95;

bool showThrusters = false;
bool cruiseControl = false;
bool parked = false;
bool hasClearedScreen = false;

public Program()
{
    Runtime.UpdateFrequency = UpdateFrequency.Update10;
    
    Me.CustomName = "System Control";
    
    List<IMyGyro> gyros2 = new List<IMyGyro>();
    GridTerminalSystem.GetBlocksOfType<IMyGyro>(gyros2);
    foreach(var gyro in gyros2)
        if(gyro.CubeGrid == Me.CubeGrid)
            gyros.Add(gyro);
    
    List<IMyShipController> cockpits = new List<IMyShipController>();
    GridTerminalSystem.GetBlocksOfType<IMyShipController>(cockpits);
    foreach(var seat in cockpits)
        if(seat.CubeGrid == Me.CubeGrid && seat.IsMainCockpit)
            cockpit = seat;
    
    var forward = cockpit.Orientation.TransformDirection(Base6Directions.Direction.Forward);
    var backward = cockpit.Orientation.TransformDirection(Base6Directions.Direction.Backward);
    
    List<IMyLandingGear> legs = new List<IMyLandingGear>();
    GridTerminalSystem.GetBlocksOfType<IMyLandingGear>(legs);
    foreach(var leg in legs)
        if(leg.CubeGrid == Me.CubeGrid && leg.IsParkingEnabled)
            parkingLegs.Add(leg);
    
    List<IMyThrust> thrusters = new List<IMyThrust>();
    GridTerminalSystem.GetBlocksOfType<IMyThrust>(thrusters);
    foreach(var thruster in thrusters)
        if(thruster.CubeGrid == Me.CubeGrid)
            if(thruster.Orientation.TransformDirection(Base6Directions.Direction.Forward) == backward)
                cruiseThrusters.Add(thruster);
            else if(thruster.Orientation.TransformDirection(Base6Directions.Direction.Forward) == forward)
                reverseThrusters.Add(thruster);
    
    cruise = (IMyLightingBlock)GridTerminalSystem.GetBlockWithName(cruiseIndicator);
    
    List<IMyTextPanel> panels = new List<IMyTextPanel>();
    GridTerminalSystem.GetBlocksOfType<IMyTextPanel>(panels);
    foreach(var panel in panels)
        if(panel.CubeGrid == Me.CubeGrid)
        {
            panel.Enabled = true;
            panel.CustomName = "Readout Screen";
            panel.ContentType = ContentType.TEXT_AND_IMAGE;
            panel.ShowInTerminal = false;
            panel.ShowInToolbarConfig = false;
            screen = panel;
            break;
        }
    
    List<IMyBatteryBlock> bats = new List<IMyBatteryBlock>();
    GridTerminalSystem.GetBlocksOfType<IMyBatteryBlock>(bats);
    foreach(var battery in bats)
        if(battery.CubeGrid == Me.CubeGrid)
        {
            battery.CustomName = "Battery "+batteries.Count;
            battery.ShowInTerminal = false;
            battery.ShowInToolbarConfig = false;
            batteries.Add(battery);
        }
    
    List<IMyPowerProducer> generators = new List<IMyPowerProducer>();
    GridTerminalSystem.GetBlocksOfType<IMyPowerProducer>(generators);
    foreach(var generator in generators)
        if(generator.CubeGrid == Me.CubeGrid && generator.BlockDefinition.SubtypeId.Contains("HydrogenEngine"))
        {
            generator.ShowInTerminal = false;
            generator.ShowInToolbarConfig = false;
            engines.Add(generator);
        }
        
    resetGyros();
    enableThrusters(true, reverseThrusters);
    enableThrusters(true, cruiseThrusters);
}

public void Save()
{
    Storage = cruisingSpeed + ";" + (showThrusters ? 1 : 0);
}

public void Main(string argument, UpdateType updateSource)
{
    hasClearedScreen = false;
    echoToScreens("Truckleweed OS Version "+version);
    if(cockpit == null)
    {
        echoToScreens("No main cockpit found");
        return;
    }
    echoToScreens("Main Cockpit: "+cockpit.CustomName);
    if(showThrusters)
    {
        foreach(var thruster in cruiseThrusters)
            echoToScreens("F: "+thruster.CustomName);
        foreach(var thruster in reverseThrusters)
            echoToScreens("R: "+thruster.CustomName);
    }
    echoToScreens("");
    if(argument.Length > 0)
    {
        var parts = argument.Split(';');
        parseCruiseSpeed(parts[0]);
        showThrusters = parts.Length > 1 && parts[1] == "ls";
        Save();
    }
    else if(Storage.Length > 0)
    {
        var parts = Storage.Split(';');
        parseCruiseSpeed(parts[0]);
        showThrusters = int.Parse(parts[1]) > 0;
    }
    
    parked = false;
    foreach(var leg in parkingLegs)
        if(leg.IsLocked)
            parked = true;
    
    handleCruiseControl();
    echoToScreens("");
    handleGyros(cockpit.RollIndicator);
    echoToScreens("");
    bool enginesOn = false;
    foreach(var engine in engines)
        if(engine.Enabled)
            enginesOn = true;
    echoToScreens("Hydrogen Engines: "+(enginesOn ? "ON" : "OFF"));
    echoToScreens("Battery Charge: "+Math.Round(getTotalChargePercent(), 2)*100+"%");
}

private void parseCruiseSpeed(string argument)
{
    try
    {
        cruisingSpeed = int.Parse(argument);
    }
    catch(Exception e)
    {
        echoToScreens("Couldn't parse cruising speed");
        cruisingSpeed = 95;
    }
}

public void echoToScreens(string text)
{
    Echo(text);
    if(screen != null)
        screen.WriteText((hasClearedScreen ? "\n" : "") + text, hasClearedScreen);
    hasClearedScreen = true;
}

private void handleCruiseControl()
{
    bool oldCruise = cruiseControl;
    
    bool hasThruster = false;
    foreach(var thruster in cruiseThrusters)
        if(thruster.Enabled && thruster.IsFunctional)
            hasThruster = true;
    
    bool hasThruster2 = false;
    foreach(var thruster in reverseThrusters)
        if(thruster.IsFunctional)
            hasThruster2 = true;
    
    cruiseControl = !parked && hasThruster && hasThruster2 && cruisingSpeed > 0 && cruise != null && cruise.Enabled && cockpit.IsUnderControl;
    
    echoToScreens("Cruise Control: "+(cruiseControl ? "ON" : "OFF"));
    if(!cockpit.IsUnderControl)
        echoToScreens(" X No pilot in main cockpit");
    if(cruisingSpeed <= 0)
        echoToScreens(" X Cruising speed set too low");
    if(!hasThruster)
        echoToScreens(" X No forward thruster available");
    if(!hasThruster2)
        echoToScreens(" X No reverse thruster available");
    if(parked)
        echoToScreens(" X Currently parked");
    if(cruise != null && cruise.Enabled != cruiseControl)
        cruise.Enabled = cruiseControl;
    
    // If cruise control state has changed...
    if(oldCruise != cruiseControl)
    {
        resetGyros();
        cockpit.SetValueBool("ControlGyros", !cruiseControl);
        enableThrusters(true, reverseThrusters);
        if(!cruiseControl)
        {
            thrustOverride(0);
            cockpit.DampenersOverride = true;
        }
    }
    
    // In cruise control, disable rotation control and maintain cruising speed efficiently
    if(cruiseControl)
    {
        double speed = cockpit.GetShipSpeed();
        cockpit.SetValueBool("ControlGyros", false);
        enableThrusters(false, reverseThrusters);
        echoToScreens(" > Target Speed: "+cruisingSpeed);
        echoToScreens(" > Current Speed: "+(int)speed);
        
        double minSpeed = Math.Max(0, cruisingSpeed - 5);
        
        if(speed <= minSpeed)
            thrustOverride(1);
        else if(speed > cruisingSpeed)
        {
            thrustOverride(0);
            enableThrusters(true, reverseThrusters);
        }
    }
}

private void enableThrusters(bool enable, List<IMyThrust> thrusters)
{
    foreach(var thruster in thrusters)
        thruster.Enabled = enable;
}

private void thrustOverride(float vol)
{
    foreach(var thruster in cruiseThrusters)
        thruster.ThrustOverride = thruster.MaxThrust * vol;
}

private void handleGyros(float roll)
{
    echoToScreens("Gyroscopes:");
    updateOrientationRef();
    int controlled = 0;
    int manual = 0;
    resetGyros();
    foreach(var gyro in gyros)
    {
        if(gyro.CubeGrid != Me.CubeGrid)
            continue;
        
        string data = gyro.CustomData;
        if((data == "manual=true" && !cruiseControl) || parked)
        {
            if(roll != 0)
            {
                gyro.GyroOverride = true;
                gyro.Yaw = -roll;
                gyro.Roll = 0F;
            }
            manual++;
            continue;
        }
        else
            controlled++;
        
        Matrix localOrientation;
        gyro.Orientation.GetMatrix(out localOrientation);
        
        var localReference = Vector3D.Transform(reference, MatrixD.Transpose(localOrientation));
        var localTarget = Vector3D.Transform(target, MatrixD.Transpose(gyro.WorldMatrix.GetOrientation()));
        
        var axis = Vector3D.Cross(localReference, localTarget);
        var angle = axis.Length();
        angle = Math.Atan2(angle, Math.Sqrt(Math.Max(0, 1 - angle * angle)));
        if(Vector3D.Dot(localReference, localTarget) < 0)
            angle = Math.PI;
        
        gyro.GyroOverride = true;
        gyro.Pitch = (float)-axis.X;
        gyro.Yaw = -roll;
        gyro.Roll = (float)-axis.Z;
    }
    echoToScreens(" "+controlled+" controlled");
    echoToScreens(" "+manual+" manual");
}

private void resetGyros()
{
    foreach(var gyro in gyros)
    {
        gyro.GyroOverride = false;
        gyro.Roll = 0F;
        gyro.Yaw = 0F;
        gyro.Pitch = 0F;
    }
}

private void updateOrientationRef()
{
    Matrix cockpitOrientation;
    cockpit.Orientation.GetMatrix(out cockpitOrientation);
    
    var quatPitch = Quaternion.CreateFromAxisAngle(cockpitOrientation.Left, 0F);
    var quatRoll = Quaternion.CreateFromAxisAngle(cockpitOrientation.Backward, 0F);
    var orientation = Vector3D.Transform(cockpitOrientation.Down, quatPitch * quatRoll);
    
    reference = orientation;
    target = cockpit.GetNaturalGravity();
    gravity = target.Length() / 10;
}

public float getTotalChargePercent()
{
    float charge = 0F;
    float maxCharge = 0F;
    foreach(var battery in batteries)
        if(battery.ChargeMode != ChargeMode.Recharge && battery.IsFunctional && battery.Enabled)
        {
            charge += battery.CurrentStoredPower;
            maxCharge += battery.MaxStoredPower;
        }
    
    return charge / maxCharge;
}