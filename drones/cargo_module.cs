// == Cargo Module OS ==
const string version = "2.0";

// Beacon
IMyBeacon beacon;
// Gyroscopes
List<IMyGyro> gyros = new List<IMyGyro>();
// Remote control
IMyRemoteControl remote;
// Thrusters mapped to their local grid directions
Dictionary<Base6Directions.Direction, List<IMyThrust>> thrustMap = new Dictionary<Base6Directions.Direction, List<IMyThrust>>();
// All thrusters
List<IMyThrust> thrusters = new List<IMyThrust>();
// All batteries
List<IMyBatteryBlock> batteries = new List<IMyBatteryBlock>();
// All LCD panels
List<IMyTextPanel> screens = new List<IMyTextPanel>();

Vector3D forwRef;
Vector3D downRef;
Vector3D gravTarget;

readonly Matrix directionMatrix = new Matrix(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
readonly Base6Directions.Direction[] vitalThrusters = 
    {Base6Directions.Direction.Up, 
    Base6Directions.Direction.Right, 
    Base6Directions.Direction.Left, 
    Base6Directions.Direction.Forward, 
    Base6Directions.Direction.Backward};

State state = State.CHARGING;
bool hasClearedScreen = false;

private enum State
{
    DOCKED,    // Connected to a stationary grid
    HAULING,    // Connected a non-stationary grid for transport
    CHARGING,    // Low power mode when undocked
    LANDING    // Used for gradual descent when unconnected
}

public Program()
{
    Runtime.UpdateFrequency = UpdateFrequency.Update10;
	
	Me.CustomName = "Cargo Module Control";
    
    List<IMyGyro> gyros2 = new List<IMyGyro>();
    GridTerminalSystem.GetBlocksOfType<IMyGyro>(gyros2);
    foreach(var gyro in gyros2)
        if(gyro.CubeGrid == Me.CubeGrid)
        {
            gyro.CustomName = "Gyroscope "+gyros.Count;
            gyro.ShowInTerminal = false;
            gyro.ShowInToolbarConfig = false;
            
            gyros.Add(gyro);
        }
    
    List<IMyRemoteControl> remotes = new List<IMyRemoteControl>();
    GridTerminalSystem.GetBlocksOfType<IMyRemoteControl>(remotes);
    foreach(var controller in remotes)
        if(controller.CubeGrid == Me.CubeGrid && controller.IsFunctional)
        {
            controller.CustomName = "Cargo Module Remote";
            remote = controller;
            break;
        }
    
    List<IMyBeacon> beacons = new List<IMyBeacon>();
    GridTerminalSystem.GetBlocksOfType<IMyBeacon>(beacons);
    foreach(var beac in beacons)
        if(beac.CubeGrid == Me.CubeGrid)
        {
            beac.CustomName = "Emergency Beacon";
            beac.Enabled = false;
            beac.ShowInTerminal = true;
            beac.ShowInToolbarConfig = false;
            beac.Radius = int.MaxValue;
            beacon = beac;
            break;
        }
    
    foreach(var direction in vitalThrusters)
        thrustMap[direction] = new List<IMyThrust>();
    if(remote != null)
    {
        List<IMyThrust> thrusters2 = new List<IMyThrust>();
        GridTerminalSystem.GetBlocksOfType<IMyThrust>(thrusters2);
        foreach(var thruster in thrusters2)
            if(thruster.CubeGrid == Me.CubeGrid)
            {
                thruster.ShowInTerminal = false;
                thruster.ShowInToolbarConfig = false;
                
                Vector3 thrustVec = getLocalFacing(thruster);
                Base6Directions.Direction localFace = Base6Directions.Direction.Forward;
                if(thrustVec == directionMatrix.Forward)
                    localFace = Base6Directions.Direction.Forward;
                else if(thrustVec == directionMatrix.Backward)
                    localFace = Base6Directions.Direction.Backward;
                else if(thrustVec == directionMatrix.Left)
                    localFace = Base6Directions.Direction.Left;
                else if(thrustVec == directionMatrix.Right)
                    localFace = Base6Directions.Direction.Right;
                else if(thrustVec == directionMatrix.Up)
                    localFace = Base6Directions.Direction.Up;
                else
                {
                    thruster.Enabled = false;
                    thruster.CustomName = "Unrecognised Thruster";
                    continue;
                }
                
                thrusters.Add(thruster);
                registerThruster(thruster, localFace);
                thruster.CustomName = "Thruster "+(Enum.GetName(typeof(Base6Directions.Direction), localFace)[0]+""+getThrusters(localFace).Count);
            }
        
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
    
    List<IMyTextPanel> panels = new List<IMyTextPanel>();
    GridTerminalSystem.GetBlocksOfType<IMyTextPanel>(panels);
    foreach(var panel in panels)
        if(panel.CubeGrid == Me.CubeGrid)
        {
            panel.Enabled = true;
            panel.CustomName = "Screen "+screens.Count;
            panel.ContentType = ContentType.TEXT_AND_IMAGE;
            panel.ShowInTerminal = false;
            panel.ShowInToolbarConfig = false;
            screens.Add(panel);
        }
}

public void Main(string argument, UpdateType updateSource)
{
    hasClearedScreen = false;
    echoToScreens("Cargo Module OS Version "+version);
    
    updateLevelling();
    updateOrientationRef();
    
    if(!diagnostic())
    {
        reset();
        return;
    }
    
    IMyShipController controller = getControllingBlock();
    bool isControllingSelf = controller == remote;
    echoToScreens("Controller: "+(isControllingSelf ? "Me" : controller.CustomName));
    bool isOnStatic = isAnyAttachedGridStatic();
    if(isOnStatic)
        state = State.DOCKED;
    
    echoToScreens("Current state: ");
    if(remote.IsUnderControl)
    {
        echoToScreens(" > Being remote controlled");
        setBatteries(ChargeMode.Auto);
        enableThrusters(true);
        resetGyros();
        foreach(var thruster in thrusters)
            thruster.ThrustOverride = 0F;
    }
    else
    {
        switch(state)
        {
            case State.DOCKED:
                echoToScreens(" > Docked");
                beacon.Enabled = false;
                enableThrusters(false);
                setBatteries(ChargeMode.Recharge);
                
                displayChargeStatus();
                
                if(!isOnStatic)
                    if(isControllingSelf)
                        setState(State.CHARGING);
                    else
                        setState(State.HAULING);
                break;
            case State.HAULING:
                echoToScreens(" > Hauling");
                echoToScreens("");
                echoToScreens("Charge: "+Math.Round(getTotalChargePercent(), 2)*100+"%");
                
                beacon.Enabled = false;
                // FIXME Identify controlling grid's downward thrust and match
                enableVerticals(true);
                enableLaterals(false);
                if(isControllingSelf)
                    setState(State.LANDING);
                break;
            case State.LANDING:
                echoToScreens(" > Landing");
                beacon.Enabled = true;
                setBatteries(ChargeMode.Auto);
                enableVerticals(true);
                enableLaterals(true);
                double altitude = getAltitude();
                double downVel = remote.GetShipVelocities().LinearVelocity.Y;
                if(altitude >= 2)
                {
                    echoToScreens("   > Altitude: "+(int)altitude);
                    enableThrusters(downVel > 1, getThrusters(Base6Directions.Direction.Up));
                    
                    return;
                }
                else
                {
                    echoToScreens("   > Speed: "+remote.GetShipSpeed());
                    enableThrusters(true);
                    foreach(var thruster in thrusters)
                        thruster.ThrustOverride = 0F;
                }
                
                if(!isControllingSelf)
                    setState(State.HAULING);
                else if(remote.GetShipSpeed() < 0.05D && altitude < 3)
                    setState(State.CHARGING);
                break;
            case State.CHARGING:
                echoToScreens(" > Charging");
                beacon.Enabled = true;
                enableThrusters(false);
                IMyBatteryBlock mostDepleted = null;
                foreach(var battery in batteries)
                    if(mostDepleted == null || mostDepleted.CurrentStoredPower > battery.CurrentStoredPower)
                        mostDepleted = battery;
                
                foreach(var battery in batteries)
                    battery.ChargeMode = battery == mostDepleted ? ChargeMode.Recharge : ChargeMode.Auto;
                
                displayChargeStatus();
                
                if(!isControllingSelf)
                    setState(State.HAULING);
                else if(remote.GetShipSpeed() > 0.3D || getAltitude() > 2)
                    setState(State.LANDING);
                break;
        }
    }
}

private void setState(State stateIn)
{
    state = stateIn;
    reset();
}

// #### UTILITY FUNCTIONS ####

public IMyShipController getControllingBlock()
{
    List<IMyCockpit> cockpits = new List<IMyCockpit>();
    GridTerminalSystem.GetBlocksOfType<IMyCockpit>(cockpits);
    foreach(var cockpit in cockpits)
        if(cockpit.IsFunctional && cockpit.IsMainCockpit)
            return cockpit;
    
    List<IMyRemoteControl> controllers = new List<IMyRemoteControl>();
    GridTerminalSystem.GetBlocksOfType<IMyRemoteControl>(controllers);
    foreach(var controller in controllers)
        if(controller.CubeGrid != Me.CubeGrid)
			if(controller.IsFunctional && controller.GetValue<bool>("MainRemoteControl"))
				return controller;
    
    return remote;
}

public bool isAnyAttachedGridStatic()
{
    List<IMyCubeBlock> blocks = new List<IMyCubeBlock>();
    GridTerminalSystem.GetBlocksOfType<IMyCubeBlock>(blocks);
    foreach(var block in blocks)
        if(block.CubeGrid.IsStatic)
            return true;
    return false;
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

public void echoToScreens(string text)
{
    Echo(text);
    addStringToDisplays(text, hasClearedScreen);
    hasClearedScreen = true;
}

public void addStringToDisplays(string text, bool append)
{
    foreach(var screen in screens)
        screen.WriteText((append ? "\n" : "") + text, append);
}

public void displayChargeStatus()
{
    
    echoToScreens("Charge status:");
    foreach(var battery in batteries)
        echoToScreens("  > "+battery.CustomName+": "+Math.Round(battery.CurrentStoredPower / battery.MaxStoredPower, 2)*100+"%"+(battery.ChargeMode == ChargeMode.Recharge ? "+" : ""));
}

public void setBatteries(ChargeMode mode)
{
    foreach(var battery in batteries)
        battery.ChargeMode = mode;
}

// Resets all vital components
private void reset()
{
    if(remote != null)
        remote.DampenersOverride = true;
    resetGyros();
    enableThrusters(true, thrusters);
    setBatteries(ChargeMode.Auto);
}

// Runs a diagnostic of all vital components and returns true if they all pass
private bool diagnostic()
{
    bool pass = true;
    if(remote == null || !remote.IsFunctional)
    {
        echoToScreens(" X Missing remote control");
        pass = false;
    }
    
    foreach(var direction in vitalThrusters)
    {
        List<IMyThrust> set = getThrusters(direction);
        string group = Enum.GetName(typeof(Base6Directions.Direction), direction);
        if(set.Count == 0)
        {
            echoToScreens(" X Missing thrusters for "+group);
            pass = false;
        }
        else
        {
            bool func = false;
            foreach(var thruster in set)
                if(thruster.IsFunctional)
                {
                    func = true;
                    break;
                }
            if(!func)
            {
                echoToScreens(" X No functional thrusters for "+group);
                pass = false;
            }
        }
    }
    
    if(gyros.Count == 0)
    {
        echoToScreens(" X Missing gyroscopes");
        pass = false;
    }
    else
    {
        bool func = false;
        foreach(var gyro in gyros)
            if(gyro.IsFunctional)
            {
                func = true;
                break;
            }
        if(!func)
        {
            echoToScreens(" X No functional gyroscopes");
            pass = false;
        }
    }
    
    return pass;
}

private float clamp(float val, float min, float max)
{
    float minVal = Math.Min(min, max);
    float maxVal = Math.Max(min, max);
    return Math.Max(minVal, Math.Min(maxVal, val));
}

// #### ROTATION HANDLING ####

// Sets gyroscopes to maintain gravity level on Pitch and Roll axises
private void updateLevelling()
{
    resetGyros();
    int gyroIndex = 0;
    foreach(var gyro in gyros)
    {
        if(gyroIndex++ % 3 > 0 && state == State.HAULING)
            continue;
        
        Matrix localOrientation;
        gyro.Orientation.GetMatrix(out localOrientation);
        var axisPR = getGyroVec(gravTarget, downRef, gyro.WorldMatrix.GetOrientation(), localOrientation);
        
        gyro.GyroOverride = true;
        gyro.Pitch = (float)-axisPR.X;
        gyro.Roll = (float)-axisPR.Z;
    }
}

private Vector3D getGyroVec(Vector3D targetVec, Vector3D refVec, MatrixD orientation, Matrix localOrientation)
{
    var localReference = Vector3D.Transform(refVec, MatrixD.Transpose(localOrientation));
    var localTarget = Vector3D.Transform(targetVec, MatrixD.Transpose(orientation));
    
    var axis = Vector3D.Cross(localReference, localTarget);
    var angle = axis.Length();
    angle = Math.Atan2(angle, Math.Sqrt(Math.Max(0, 1 - angle * angle)));
    if(Vector3D.Dot(localReference, localTarget) < 0)
        angle = Math.PI;
    
    return axis;
}

// Updates the orientation values used for navigation and levelling
private void updateOrientationRef()
{
    Matrix shipOrientation;
    remote.Orientation.GetMatrix(out shipOrientation);
    
    var quatPitch = Quaternion.CreateFromAxisAngle(shipOrientation.Left, 0F);
    var quatRoll = Quaternion.CreateFromAxisAngle(shipOrientation.Backward, 0F);
    var orientation = Vector3D.Transform(shipOrientation.Forward, quatPitch * quatRoll);
    
    forwRef = orientation;
    downRef = Vector3D.Transform(shipOrientation.Down, quatPitch * quatRoll);
    gravTarget = remote.GetNaturalGravity();
}

// Resets gyroscopes to prevent further rotation after state change
private void resetGyros()
{
    foreach(var gyro in gyros)
    {
        gyro.GyroOverride = false;
        gyro.Pitch = 0F;
        gyro.Yaw = 0F;
        gyro.Roll = 0F;
    }
}

// #### VELOCITY HANDLING ####

private void registerThruster(IMyThrust thruster, Base6Directions.Direction direction)
{
    List<IMyThrust> set = thrustMap[direction];
    set.Add(thruster);
    thrustMap[direction] = set;
}

private List<IMyThrust> getThrusters(Base6Directions.Direction direction)
{
    return thrustMap[direction];
}

private Vector3 getLocalFacing(IMyCubeBlock block)
{
    Matrix fromGridToReference;
    remote.Orientation.GetMatrix(out fromGridToReference);
    Matrix.Transpose(ref fromGridToReference, out fromGridToReference);
    
    Matrix fromThrusterToGrid;
    block.Orientation.GetMatrix(out fromThrusterToGrid);
    return Vector3.Transform(fromThrusterToGrid.Backward, fromGridToReference);
}

private double getAltitude()
{
    double altSea = 0D;
    double altVox = 0D;
    remote.TryGetPlanetElevation(MyPlanetElevation.Sealevel, out altSea);
    remote.TryGetPlanetElevation(MyPlanetElevation.Surface, out altVox);
    return Math.Min(altSea, altVox);
}

private void enableVerticals(bool enable)
{
    enableThrusters(enable, getThrusters(Base6Directions.Direction.Up));
}

private void enableLaterals(bool enable)
{
    enableThrusters(enable, getThrusters(Base6Directions.Direction.Forward));
    enableThrusters(enable, getThrusters(Base6Directions.Direction.Backward));
    enableThrusters(enable, getThrusters(Base6Directions.Direction.Left));
    enableThrusters(enable, getThrusters(Base6Directions.Direction.Right));
}

private void enableThrusters(bool enable)
{
    enableThrusters(enable, thrusters);
}

// Sets all blocks in the given list to be on/off
private void enableThrusters(bool enable, List<IMyThrust> blocks)
{
    foreach(var block in blocks)
        block.Enabled = enable;
}