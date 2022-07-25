// == Beagle Shuttle OS ==
const string version = "1.0";
const String spinning = "-\\|/";
const int max_scan = 1001;

// Shuttle model
public static Shuttle model;
readonly static Matrix directionMatrix = new Matrix(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
private static Dictionary<Operation, ShipOperation> OPERATION_MAP = new Dictionary<Operation, ShipOperation>();

const String namePrefix = "BGL";
private static Orientation currentOri = Orientation.MANUAL;
private static Operation currentOp = Operation.NONE;
public static bool ccOn = false;
public static int ccSpeed = 50;

private static List<IMyTextPanel> allScreens = new List<IMyTextPanel>();
private List<IMyTextPanel> screensStatus = new List<IMyTextPanel>();
private List<IMyTextPanel> screensInventory = new List<IMyTextPanel>();
public enum ScreenType
{
    INV,
    STATUS
}

public enum Orientation
{
    MANUAL,
    AWAY,
    PERP
}
public enum Operation
{
    NONE,
    LAUNCH,
    LAND,
    ELAND
}
private static Orientation orientationFromName(String nameIn)
{
    nameIn = nameIn.ToLower();
    foreach(Orientation style in Enum.GetValues(typeof(Orientation)))
        if(Enum.GetName(typeof(Orientation), style).ToLower() == nameIn)
            return style;
    return Orientation.MANUAL;
}
private static Operation operationFromName(String nameIn)
{
    nameIn = nameIn.ToLower();
    foreach(Operation style in Enum.GetValues(typeof(Operation)))
        if(Enum.GetName(typeof(Operation), style).ToLower() == nameIn)
            return style;
    return Operation.NONE;
}

private static int ticksRunning;
private bool hasClearedScreen;

public Program()
{
    Runtime.UpdateFrequency = UpdateFrequency.Update10;
    OPERATION_MAP[Operation.LAUNCH] = new OperationLaunch();
    OPERATION_MAP[Operation.LAND] = new OperationLand();
    OPERATION_MAP[Operation.ELAND] = new OperationLowPower();
    
    Me.CubeGrid.CustomName = namePrefix;
    Me.CustomName = namePrefix+" Control";
    
    model = new Shuttle(Me, this);
    loadStorage(Storage);
    
    List<IMyTextPanel> LCDs = new List<IMyTextPanel>();
    GridTerminalSystem.GetBlocksOfType<IMyTextPanel>(LCDs);
    foreach(var LCD in LCDs)
        if(LCD.CubeGrid == Me.CubeGrid)
        {
            LCD.CustomName = namePrefix + " Info Screen "+allScreens.Count;
            string data = LCD.CustomData.ToLower();
            if(data == "inv")
                screensInventory.Add(LCD);
            else if(data == "status")
                screensStatus.Add(LCD);
            allScreens.Add(LCD);
        }
}

public void Save()
{
    Storage = String.Join(";", Enum.GetName(typeof(Operation), currentOp), Enum.GetName(typeof(Orientation), currentOri));
    Storage += "|" + String.Join(";", ccOn ? 1 : 0, ccSpeed);
}

public void Main(string argument, UpdateType updateSource)
{
    hasClearedScreen = false;
    ++ticksRunning;
    
    if(argument.Length > 0)
        parseArg(argument);
    
    model.tick(Echo);
    if(!model.diagnostic(Echo))
    {
        Echo("Diagnostic failed");
        reset();
        model.setBroadcast("ERROR");
        return;
    }
    
    handleScreensStatus();
    handleScreensInventory();
    
    if(!noOperation())
    {
        ShipOperation op = OPERATION_MAP.ContainsKey(currentOp) ? OPERATION_MAP[currentOp] : null;
        op.tick(model, echoToStatus);
        if(op.invalid(model))
            setOperation(Operation.NONE);
    }
    else if(ccOn)
    {
        Vector3D forward = new Vector3D(0, 0, -1);
        forward = Vector3D.Multiply(model.toWorld(forward), ccSpeed);
        model.setTargetVelocity(forward);
    }
    if(currentOri != Orientation.MANUAL && model.getGravity().Length() == 0)
        setOrientation(Orientation.MANUAL);
    
    double grav = model.getGravity().Length();
    if(currentOp != Operation.ELAND && grav > 0)
        if(model.getAvailableChargePercent() < 0.2F) // TODO Include Hydrogen tank status check
            setOperation(Operation.ELAND);
    
    Save();
}

public void parseArg(String argument)
{
    String val = argument.ToLower();
    if(val == "reset")
        reset();
    else if(val == "remodel")
        model = new Shuttle(Me, this);
    else if(val.StartsWith("set_ori="))
        setOrientation(orientationFromName(val.Split('=')[1].ToLower()));
    else if(val.StartsWith("set_op="))
        setOperation(operationFromName(val.Split('=')[1].ToLower()));
    else if(val == "cc_off")
        enableCruiseControl(false);
    else if(val.StartsWith("cc_on") && noOperation())
    {
        enableCruiseControl(true);
        if(val.StartsWith("cc_on="))
            ccSpeed = (int)clamp(int.Parse(val.Split('=')[1]), 20, 140);
    }
    else if(val == "launch")
        parseArg("set_op=LAUNCH");
    else if(val == "land")
        parseArg("set_op=LAND");
    
    Save();
}

public void loadStorage(String memory)
{
    reset();
    String[] lines = memory.ToLower().Split('|');
    int entries = lines.Length;
    
    // Operating data
    String[] data = lines[0].Split(';');
    if(data.Length == 2)
    {
        setOperation(operationFromName(data[0]));
        currentOri = orientationFromName(data[1]);
    }
    if(--entries <= 0) return;
    
    data = lines[1].Split(';');
    if(data.Length == 2)
    {
        ccOn = int.Parse(data[0]) > 0;
        ccSpeed = int.Parse(data[1]);
    }
    if(--entries <= 0) return;
}

public void handleScreensStatus()
{
    hasClearedScreen = false;
    echoToStatus("Beagle OS Version "+version+" "+getSpinning());
    echoToStatus("Orientation: "+Enum.GetName(typeof(Orientation), currentOri));
    echoToStatus("Operation: "+Enum.GetName(typeof(Operation), currentOp));
    echoToStatus("");
    if(ccOn)
        echoToStatus("Cruise Control Active: "+ccSpeed+"m/s");
    echoToStatus("Velocity: "+Math.Round(model.getShipVelocities().Length(), 2)+"m/s");
    echoToStatus("");
    echoToStatus("Atmosphere: "+Math.Round(model.getAtmosphere(), 2)+"atm");
    double grav = model.getGravity().Length();
    echoToStatus("Gravity: "+Math.Round(grav / 10, 2)+"g");
    if(grav > 0)
    {
        double alt = Math.Round(model.getAltitude(), 2);
        echoToStatus("Altitude: "+(alt > 1000 ? ">1km" : alt+"m"));
    }
	if(!noOperation())
		echoToStatus("Operation Log:");
}

public void handleScreensInventory()
{
    hasClearedScreen = false;
    echoToInventory("Power: "+Math.Round(model.getTotalChargePercent()*100, 2)+"%");
}

public void echoToScreens(string text) { echoToScreens(text, allScreens); }
public void echoToStatus(string text) { echoToScreens(text, screensStatus); }
public void echoToInventory(string text) { echoToScreens(text, screensInventory); }
private void echoToScreens(string text, List<IMyTextPanel> screensIn)
{
    Echo(text);
    foreach(var screen in screensIn)
        if(screen != null && screen.IsFunctional && screen.Enabled)
            screen.WriteText((hasClearedScreen ? "\n" : "") + text, hasClearedScreen);
    hasClearedScreen = true;
}

private void reset()
{
    setOperation(Operation.NONE);
    setOrientation(Orientation.MANUAL);
    enableCruiseControl(false);
    Save();
}

public static Operation getOperation(){ return currentOp; }
public static bool noOperation(){ return currentOp == Operation.NONE; }
public static void setOperation(Operation opIn)
{
    if(!noOperation())
    {
        ShipOperation current = OPERATION_MAP.ContainsKey(currentOp) ? OPERATION_MAP[currentOp] : null;
        if(current != null)
            current.exit(model);
    }
    else if(model.getGravity().Length() == 0)
        return;
    
    ShipOperation next = OPERATION_MAP.ContainsKey(opIn) ? OPERATION_MAP[opIn] : null;
    if(next != null)
        next.enter(model);
    
    currentOp = opIn;
    if(!noOperation())
        enableCruiseControl(false);
}

public static Orientation getOrientation(){ return currentOri; }
public static void setOrientation(Orientation oriIn)
{
    if(oriIn != Orientation.MANUAL)
        if(model.collisionAlarm() || model.getGravity().Length() == 0)
            return;
    
    currentOri = oriIn;
}

public static void enableCruiseControl(bool valIn)
{
    if(ccOn && !valIn)
        model.clearTargetVelocity();
    ccOn = valIn;
}

// #### UTILITY FUNCTIONS ####
public static char getSpinning()
{
    return spinning[ticksRunning % spinning.Length];
}

public static Vector3 getLocalFacing(IMyCubeBlock block, IMyShipController remote)
{
    Matrix fromGridToReference;
    remote.Orientation.GetMatrix(out fromGridToReference);
    Matrix.Transpose(ref fromGridToReference, out fromGridToReference);
    
    Matrix fromThrusterToGrid;
    block.Orientation.GetMatrix(out fromThrusterToGrid);
    return Vector3.Transform(fromThrusterToGrid.Backward, fromGridToReference);
}

public static float clamp(float val, float min, float max)
{
    float minVal = Math.Min(min, max);
    float maxVal = Math.Max(min, max);
    return Math.Max(minVal, Math.Min(maxVal, val));
}

public class Shuttle
{
    // Remote control
    private IMyShipController mainControl;
    // Antenna
    private IMyRadioAntenna antenna;
    // Gyroscopes
    private List<IMyGyro> gyros = new List<IMyGyro>();
    // Terrain sensor
    private IMySensorBlock sensor;
    private IMyCameraBlock camera;
    // All downward connectors
    private List<IMyShipConnector> connectors = new List<IMyShipConnector>();
    // All parachute hatches
    private List<IMyParachute> parachutes = new List<IMyParachute>();
    // All batteries
    private List<IMyBatteryBlock> batteries = new List<IMyBatteryBlock>();
    // All thrusters
    private List<IMyThrust> thrusters = new List<IMyThrust>();
    private Dictionary<Base6Directions.Direction, List<IMyThrust>> thrustMap = new Dictionary<Base6Directions.Direction, List<IMyThrust>>();
    private PID pidX = new PID(), pidY = new PID(), pidZ = new PID();
    private Vector3D targetVelocity = new Vector3D(0, 0, 0);
    
    private Vector3D downRef;
    private Vector3D gravTarget;
    private double latestAltitude = 0;
    
    private String broadcastMessage = "";
    
    public Shuttle(IMyProgrammableBlock me, MyGridProgram program)
    {
        List<IMyShipController> cockpits = new List<IMyShipController>();
        program.GridTerminalSystem.GetBlocksOfType<IMyShipController>(cockpits);
        foreach(var cock in cockpits)
            if(cock.CubeGrid == me.CubeGrid && cock.IsFunctional && cock.IsMainCockpit)
            {
                cock.CustomName = namePrefix + " Cockpit";
                cock.DampenersOverride = true;
                mainControl = cock;
                break;
            }
        
        List<IMyRadioAntenna> radios = new List<IMyRadioAntenna>();
        program.GridTerminalSystem.GetBlocksOfType<IMyRadioAntenna>(radios);
        foreach(var radio in radios)
            if(radio.CubeGrid == me.CubeGrid && radio.IsFunctional)
            {
                radio.CustomName = namePrefix+" Antenna";
                radio.EnableBroadcasting = true;
                antenna = radio;
                break;
            }
        
        List<IMyGyro> gyros2 = new List<IMyGyro>();
        program.GridTerminalSystem.GetBlocksOfType<IMyGyro>(gyros2);
        foreach(var gyro in gyros2)
            if(gyro.CubeGrid == me.CubeGrid)
            {
                gyro.CustomName = namePrefix+" Gyro "+gyros.Count;
                gyro.Enabled = true;
                gyro.ShowInTerminal = false;
                gyro.ShowInToolbarConfig = false;
                gyros.Add(gyro);
            }
        
        foreach(var direction in Base6Directions.EnumDirections)
            thrustMap[direction] = new List<IMyThrust>();
        if(mainControl != null)
        {
            List<IMyShipConnector> connect = new List<IMyShipConnector>();
            program.GridTerminalSystem.GetBlocksOfType<IMyShipConnector>(connect);
            foreach(var connector in connect)
                if(connector.CubeGrid == me.CubeGrid && getLocalFacing(connector, mainControl) == directionMatrix.Up)
                {
                    connector.CustomName = namePrefix+" Connector "+connectors.Count;
                    connector.Enabled = true;
                    connector.ShowInToolbarConfig = false;
                    connector.ShowInTerminal = false;
                    
                    connectors.Add(connector);
                }
            
            List<IMyThrust> thrusters2 = new List<IMyThrust>();
            program.GridTerminalSystem.GetBlocksOfType<IMyThrust>(thrusters2);
            foreach(var thruster in thrusters2)
            {
                if(thruster.CubeGrid == me.CubeGrid)
                {
                    thruster.ShowInTerminal = false;
                    thruster.ShowInToolbarConfig = false;
                    
                    Vector3 thrustVec = getLocalFacing(thruster, mainControl);
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
                    else if(thrustVec == directionMatrix.Down)
                        localFace = Base6Directions.Direction.Down;
                    else
                    {
                        thruster.Enabled = false;
                        thruster.CustomName = namePrefix+" Unrecognised Thruster";
                        continue;
                    }
                    
                    thrusters.Add(thruster);
                    registerThruster(thruster, localFace);
                    thruster.CustomName = namePrefix+" Thruster "+(Enum.GetName(typeof(Base6Directions.Direction), localFace)[0]+""+getThrusters(localFace).Count);
                }
            }
            
            List<IMyCameraBlock> cameras = new List<IMyCameraBlock>();
            program.GridTerminalSystem.GetBlocksOfType<IMyCameraBlock>(cameras);
            foreach(var cam in cameras)
                if(cam.CubeGrid == me.CubeGrid && getLocalFacing(cam, mainControl) == directionMatrix.Up)
                {
                    cam.CustomName = namePrefix+" Altitude Sensor";
                    cam.Enabled = true;
                    cam.EnableRaycast = true;
                    camera = cam;
                    break;
                }
        }
        
        List<IMyParachute> chutes = new List<IMyParachute>();
        program.GridTerminalSystem.GetBlocksOfType<IMyParachute>(chutes);
        foreach(var chute in chutes)
            if(chute.CubeGrid == me.CubeGrid)
            {
                chute.CustomName = namePrefix + " Parachute "+parachutes.Count;
                chute.Enabled = true;
                chute.ShowInTerminal = false;
                chute.ShowInToolbarConfig = false;
                
                parachutes.Add(chute);
            }
        
        List<IMySensorBlock> sensors = new List<IMySensorBlock>();
        program.GridTerminalSystem.GetBlocksOfType<IMySensorBlock>(sensors);
        foreach(var sense in sensors)
            if(sense.CubeGrid == me.CubeGrid && sense.IsFunctional)
            {
                sense.CustomName = namePrefix+" Collision Sensor";
                sense.Enabled = true;
                
                Vector3 faceVec = getLocalFacing(sense, mainControl);
                sense.BackExtend = faceVec.Y <= 0 ? 20 : 0;
                sense.FrontExtend = faceVec.Y >= 0 ? 20 : 0;
                sense.RightExtend = sense.LeftExtend = sense.TopExtend = sense.BottomExtend = int.MaxValue;
                
                sense.DetectAsteroids = true;
                sense.DetectFloatingObjects = false;
                sense.DetectLargeShips = true;
                sense.DetectStations = true;
                sense.DetectPlayers = false;
                sense.DetectSmallShips = false;
                sense.DetectSubgrids = false;
                
                sense.ShowInToolbarConfig = false;
                sensor = sense;
                break;
            }
        
        List<IMyBatteryBlock> bats = new List<IMyBatteryBlock>();
        program.GridTerminalSystem.GetBlocksOfType<IMyBatteryBlock>(bats);
        foreach(var battery in bats)
            if(battery.CubeGrid == me.CubeGrid)
            {
                battery.CustomName = namePrefix+" Battery "+batteries.Count;
                battery.ShowInTerminal = false;
                battery.ShowInToolbarConfig = false;
                batteries.Add(battery);
            }
    }
    
    public void tick(Action<string> echo)
    {
        if(antenna != null && antenna.IsFunctional)
        {
            if(broadcastMessage.Length > 0)
                antenna.HudText = broadcastMessage;
            else
                antenna.HudText = "";
            antenna.ShowShipName = broadcastMessage.Length == 0;
        }
        
        updateOrientationRef();
        updateLevelling();
        
        // Total mass of ship (kg)
        double mass = mainControl.CalculateShipMass().TotalMass;
        
        // Downward gravity relative to ship (m/s)
        double grav = toLocal(getGravity()).Length();
        
        // Velocity relative to ship (m/s)
        Vector3D velocity = toLocal(mainControl.GetShipVelocities().LinearVelocity);
        
        // Target velocity relative to ship (m/s)
        Vector3D targetVel = toLocal(targetVelocity);
        
        handleThrust(targetVel.X, velocity.X, pidX, mass, getThrusters(Base6Directions.Direction.Right), getThrusters(Base6Directions.Direction.Left));
        handleThrust(targetVel.Y, velocity.Y, pidY, mass, getThrusters(Base6Directions.Direction.Up), Math.Abs(grav)*mass);
        handleThrust(targetVel.Z, velocity.Z, pidZ, mass, getThrusters(Base6Directions.Direction.Backward), getThrusters(Base6Directions.Direction.Forward));
    }
    
    // Runs a diagnostic of all vital components and returns true if they all pass
    public bool diagnostic(Action<string> echo)
    {
        bool pass = true;
        if(mainControl == null || !mainControl.IsFunctional)
        {
            echo(" X Missing remote control");
            pass = false;
        }
        
        foreach(var direction in Base6Directions.EnumDirections)
        {
            List<IMyThrust> set = getThrusters(direction);
            string group = Enum.GetName(typeof(Base6Directions.Direction), direction);
            if(set.Count == 0)
            {
                echo(" X Missing thrusters for "+group);
                pass = false;
            }
            else
            {
                bool func = false;
                foreach(var thruster in set)
                    if(thruster != null && thruster.IsFunctional)
                    {
                        func = true;
                        break;
                    }
                if(!func)
                {
                    echo(" X No functional thrusters for "+group);
                    pass = false;
                }
            }
        }
        
        if(gyros.Count == 0)
        {
            echo(" X Missing gyroscopes");
            pass = false;
        }
        else
        {
            bool func = false;
            foreach(var gyro in gyros)
                if(gyro != null && gyro.IsFunctional)
                {
                    func = true;
                    break;
                }
            if(!func)
            {
                echo(" X No functional gyroscopes");
                pass = false;
            }
        }
        
        if(sensor == null || !sensor.IsFunctional)
        {
            echo(" X No terrain collision sensor");
            pass = false;
        }
        
        if(connectors.Count == 0)
            echo(" ? No connectors");
        else
        {
            bool func = false;
            foreach(var connector in connectors)
                if(connector != null && connector.IsFunctional)
                {
                    func = true;
                    break;
                }
            if(!func)
                echo(" ? No functional connector");
        }
        
        if(camera == null || !camera.IsFunctional)
            echo(" ? Missing altitude camera");
        
        if(antenna == null || !antenna.IsFunctional)
            echo(" ? Missing antenna");
        
        return pass;
    }
    
    public Vector3D getGravity(){ return mainControl == null ? new Vector3D(0, 0, 0) : mainControl.GetNaturalGravity(); }
    
    public Vector3D toLocal(Vector3D vectorIn){ return Vector3D.TransformNormal(vectorIn, MatrixD.Transpose(mainControl.WorldMatrix)); }
    public Vector3D toWorld(Vector3D vectorIn){ return Vector3D.TransformNormal(vectorIn, mainControl.WorldMatrix); }
    
    public void setBroadcast(String messageIn)
    {
        if(messageIn.Length > 0 && !messageIn.StartsWith(namePrefix))
            messageIn = namePrefix + " " + messageIn;
        broadcastMessage = messageIn;
    }
    
    public float getAtmosphere()
    {
        float best = 0F;
        foreach(var chute in parachutes)
            if(chute != null && chute.IsFunctional)
                best = Math.Max(best, chute.Atmosphere);
        return best;
    }
    
    public double getAltitude()
    {
        if(camera == null || !(camera.Enabled && camera.IsFunctional))
        {
            // If camera is absent or non-functional, use cockpit reading
            // WARNING! This reading is drawn from voxels at worldgen and can be very inaccurate
            return getControllerAlt();
        }
        
        camera.EnableRaycast = true;
        if(camera.CanScan(max_scan))
        {
            MyDetectedEntityInfo scan = camera.Raycast(max_scan);
            if(scan.IsEmpty())
                latestAltitude = max_scan + 1;
            else if(scan.HitPosition.HasValue)
                latestAltitude = (camera.GetPosition() - scan.HitPosition.Value).Length();
        }
        
        if(latestAltitude >= max_scan)
            return getControllerAlt();
        return latestAltitude;
    }
    
    private double getControllerAlt()
    {
        double altSea = 0D;
        double altVox = 0D;
        mainControl.TryGetPlanetElevation(MyPlanetElevation.Sealevel, out altSea);
        mainControl.TryGetPlanetElevation(MyPlanetElevation.Surface, out altVox);
        return Math.Min(altSea, altVox);
    }
    
    public bool collisionAlarm(){ return sensor != null && sensor.Enabled && sensor.IsActive; }
    
    public void setConnector(bool on)
    {
        foreach(var connector in connectors)
            if(connector != null)
                connector.Enabled = on;
    }
    public bool isConnected()
    {
        return getConnectorStatus() == MyShipConnectorStatus.Connected;
    }
    public MyShipConnectorStatus getConnectorStatus()
    {
        MyShipConnectorStatus bestStatus = MyShipConnectorStatus.Unconnected;
        foreach(var connector in connectors)
        {
            if(connector == null || !connector.IsFunctional || !connector.Enabled)
                continue;
            switch(connector.Status)
            {
                case MyShipConnectorStatus.Connected:
                    bestStatus = connector.Status;
                    break;
                case MyShipConnectorStatus.Connectable:
                    if(bestStatus == MyShipConnectorStatus.Unconnected)
                        bestStatus = connector.Status;
                    break;
                case MyShipConnectorStatus.Unconnected:
                    break;
            }
        }
        return bestStatus;
    }
    
    // Sets gyroscopes to maintain gravity level on Pitch and Roll axises
    private void updateLevelling()
    {
        resetGyros();
        if(getOrientation() == Orientation.MANUAL)
            return;
        
        foreach(var gyro in gyros)
        {
            if(gyro == null || !gyro.Enabled || !gyro.IsFunctional) continue;
            gyro.GyroOverride = true;
            Matrix localOrientation;
            gyro.Orientation.GetMatrix(out localOrientation);
            
            var axisPR = getGyroVec(gravTarget, downRef, gyro.WorldMatrix.GetOrientation(), localOrientation);
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
        mainControl.Orientation.GetMatrix(out shipOrientation);
        
        var quatPitch = Quaternion.CreateFromAxisAngle(shipOrientation.Left, 0F);
        var quatRoll = Quaternion.CreateFromAxisAngle(shipOrientation.Backward, 0F);
        
        downRef = Vector3D.Transform(getOrientation() == Orientation.AWAY ? shipOrientation.Backward : shipOrientation.Down, quatPitch * quatRoll);
        gravTarget = mainControl.GetNaturalGravity();
    }

    // Resets gyroscopes to prevent further rotation after state change
    private void resetGyros()
    {
        foreach(var gyro in gyros)
        {
            if(gyro == null || !gyro.IsFunctional) continue;
            gyro.GyroOverride = false;
            gyro.Pitch = 0F;
            gyro.Yaw = 0F;
            gyro.Roll = 0F;
        }
    }

    public float getAvailableChargePercent()
    {
        float charge = 0F;
        float maxCharge = 0F;
        foreach(var battery in batteries)
            if(battery != null && battery.IsFunctional && battery.Enabled && battery.ChargeMode != ChargeMode.Recharge)
            {
                charge += battery.CurrentStoredPower;
                maxCharge += battery.MaxStoredPower;
            }
        
        return charge / maxCharge;
    }
    
    public float getTotalChargePercent()
    {
        float charge = 0F;
        float maxCharge = 0F;
        foreach(var battery in batteries)
            if(battery != null && battery.IsFunctional)
            {
                charge += battery.CurrentStoredPower;
                maxCharge += battery.MaxStoredPower;
            }
        
        return charge / maxCharge;
    }
    
    public void doCharging()
    {
        IMyBatteryBlock lowest = null;
        float lowestCharge = float.MaxValue;
        foreach(var battery in batteries)
        {
            battery.ChargeMode = ChargeMode.Auto;
            float charge = battery.CurrentStoredPower / battery.MaxStoredPower;
            if(charge < lowestCharge)
            {
                lowest = battery;
                lowestCharge = charge;
            }
        }
        if(batteries.Count > 1)
            lowest.ChargeMode = ChargeMode.Recharge;
    }
    
    public void resetBatteries()
    {
        foreach(var battery in batteries)
            if(battery != null)
            {
                battery.Enabled = true;
                battery.ChargeMode = ChargeMode.Auto;
            }
    }
    
    public Vector3D getShipVelocities(){ return mainControl.GetShipVelocities().LinearVelocity; }
    public Vector3D getShipRotations(){ return mainControl.GetShipVelocities().AngularVelocity; }
    
    private void registerThruster(IMyThrust thruster, Base6Directions.Direction direction)
    {
        List<IMyThrust> set = thrustMap[direction];
        set.Add(thruster);
        thrustMap[direction] = set;
    }
    
    public void setTargetVelocity(Vector3D vel)
    {
        targetVelocity = vel;
    }
    public void clearTargetVelocity(){ targetVelocity = new Vector3D(0, 0, 0); }
    
    private List<IMyThrust> getThrusters(Base6Directions.Direction face)
    {
        return thrustMap[face];
    }
    
    private void resetThrusters(List<IMyThrust> group)
    {
        foreach(var thruster in group)
        {
            if(thruster == null || !thruster.IsFunctional) continue;
            thruster.Enabled = thruster.MaxThrust > 0;
            thruster.ThrustOverride = 0F;
        }
    }
    
    private void handleThrust(double target, double current, PID pid, double mass, List<IMyThrust> group, double offset)
    {
        handleThrust(target, current, pid, mass, group, new List<IMyThrust>(), offset);
    }
    
    private void handleThrust(double target, double current, PID pid, double mass, List<IMyThrust> groupA, List<IMyThrust> groupB)
    {
        handleThrust(target, current, pid, mass, groupA, groupB, 0D);
    }
    
    private void handleThrust(double target, double current, PID pid, double mass, List<IMyThrust> groupA, List<IMyThrust> groupB, double offset)
    {
        resetThrusters(groupA);
        resetThrusters(groupB);
        
        int tallyA = 0;
        float avgThrustA = 0F;
        foreach(var thruster in groupA)
            if(thruster != null && thruster.IsFunctional)
            {
                if(thruster.MaxThrust == 0F)
                {
                    thruster.Enabled = false;
                    continue;
                }
                tallyA++;
                avgThrustA += thruster.MaxThrust;
            }
        avgThrustA /= tallyA;
        
        int tallyB = 0;
        float avgThrustB = 0F;
        foreach(var thruster in groupB)
            if(thruster != null && thruster.IsFunctional)
            {
                if(thruster.MaxThrust == 0F)
                {
                    thruster.Enabled = false;
                    continue;
                }
                tallyB++;
                avgThrustB += thruster.MaxThrust;
            }
        avgThrustB /= tallyB;
        
        if(target != 0F)
        {
            double pidVal = pid.calculate(target, current) * mass;
            
            // Total thrust needed (N)
            double totalThrust = offset + pidVal;
            foreach(var thruster in groupA)
            {
                if(thruster == null || !thruster.IsFunctional) continue;
                thruster.Enabled = totalThrust > 0F;
                thruster.ThrustOverride = thruster.Enabled ? (float)(Math.Abs(totalThrust) / tallyA) * (thruster.MaxThrust / avgThrustA) : 0F;
            }
            
            totalThrust -= offset;
            foreach(var thruster in groupB)
            {
                if(thruster == null || !thruster.IsFunctional) continue;
                thruster.Enabled = totalThrust < 0F;
                thruster.ThrustOverride = thruster.Enabled ? (float)(Math.Abs(totalThrust) / tallyB) * (thruster.MaxThrust / avgThrustB) : 0F;
            }
        }
        else
        {
            // Just use regular dampening if we just want to not move
            resetThrusters(groupA);
            resetThrusters(groupB);
            pid.reset();
        }
    }
    
    public class PID
    {
        // Interval time
        private double DT = 1D;
        // Proportional gain
        private double KP = 1D;
        //  Integral gain
        private double KI = 1D;
        // Derivative gain
        private double KD = 1D;
        
        private double integral = 0D;
        private double prevError = 0D;
        
        public PID(double rateIn, double gainIn) : this(rateIn, gainIn, gainIn, gainIn){ }
        public PID() : this(1D,2.5D){ }
        public PID(double rate, double propGain, double intGain, double derGain)
        {
            DT = rate;
            KP = propGain;
            KI = intGain;
            KD = derGain;
        }
        
        public double calculate(double setPoint, double pv)
        {
            double error = setPoint - pv;
            
            double propOut = KP * error;
            
            integral += error * DT;
            double intOut = KI * integral;
            
            double derivative = (error - prevError) / DT;
            double derOut = KD * derivative;
            
            double output = propOut + intOut + derOut;
            
            prevError = error;
            return output;
        }
        
        public void reset()
        {
            prevError = 0D;
            integral = 0D;
        }
    }
}

public abstract class ShipOperation
{
    private Controls[] affectedComponents = new Controls[0];
    
    protected ShipOperation(Controls[] affectedIn)
    {
        affectedComponents = affectedIn;
    }
    
    // Performs any basic initial settings this state needs
    public virtual void enter(Shuttle model){ }
    // Performs the functions of this state
    public abstract void tick(Shuttle model, Action<string> echo);
    // Returns true if a state change should occur, ie. this state's work is finished and it can safely stop
    public abstract bool invalid(Shuttle model);
    // Resets any changes caused by this state and returns the state names that should be moved into
    public virtual void exit(Shuttle model)
    {
        resetModel(model);
        onExit(model);
    }
    public virtual void onExit(Shuttle model){ }
    
    protected virtual void resetModel(Shuttle model)
    {
        foreach(var component in affectedComponents)
            switch(component)
            {
                case Controls.THRUSTERS: model.clearTargetVelocity(); break;
                case Controls.GYROS: setOrientation(Orientation.MANUAL); break;
                case Controls.CONNECTOR: model.setConnector(true); break;
                case Controls.BATTERIES: model.resetBatteries(); break;
                case Controls.ANTENNA: model.setBroadcast(null); break;
            }
    }
    
    public enum Controls
    {
        THRUSTERS,
        GYROS,
        CONNECTOR,
        BATTERIES,
        ANTENNA
    }
}

public class OperationLaunch : ShipOperation
{
    public OperationLaunch() : base(new Controls[]{Controls.THRUSTERS, Controls.CONNECTOR}){ }
    
    public override void enter(Shuttle model)
    {
        model.setConnector(false);
    }
    
    public override void tick(Shuttle model, Action<string> echo)
    {
        int altitude = (int)model.getAltitude();
        echo("   > Altitude: "+altitude);
        
        if(model.isConnected())
            model.setConnector(false);
        else if(altitude > 2)
        {
            Vector3D direction = Vector3D.Multiply(model.getGravity(), -1);
            direction.Normalize();
            
            bool collision = model.collisionAlarm();
            model.setTargetVelocity(Vector3D.Multiply(direction, collision ? 2D : 140D));
            
            if(!collision)
                setOrientation(model.getAtmosphere() == 0F ? Orientation.AWAY : Orientation.PERP);
        }
        else
            model.clearTargetVelocity();
    }
    
    public override bool invalid(Shuttle model)
    {
        return model.getGravity().Length() == 0;
    }
}

public class OperationLand : ShipOperation
{
    public OperationLand() : base(new Controls[]{Controls.THRUSTERS, Controls.GYROS}){ }
    protected OperationLand(Controls[] controlsIn) : base(controlsIn){ }
    
    public override void tick(Shuttle model, Action<string> echo)
    {
        int altitude = (int)model.getAltitude();
        
        if(altitude > 2 && !model.isConnected())
        {
            Vector3D direction = model.getGravity();
            direction.Normalize();
            
            bool collision = model.collisionAlarm();
            double speed = Math.Min(100, altitude / 5);
            if(altitude <= 500)
                speed = (double)(altitude / 10);
            if(collision || speed < 2D)
                speed = 2D;
            
            if(!collision)
                setOrientation(model.getAtmosphere() < 0.3F ? Orientation.AWAY : Orientation.PERP);
            
            double velocity = model.getShipVelocities().Length();
            echo("   > Target: "+speed+"m/s");
            model.setTargetVelocity(Vector3D.Multiply(direction, speed));
            if(velocity > speed * 2)
                model.clearTargetVelocity();
        }
        else
            model.clearTargetVelocity();
    }
    
    public override bool invalid(Shuttle model)
    {
        return model.getAltitude() <= 2 || model.isConnected();
    }
}

public class OperationLowPower : OperationLand
{
    public OperationLowPower() : base(new Controls[]{Controls.THRUSTERS, Controls.ANTENNA}){ }
    
    public override void tick(Shuttle model, Action<string> echo)
    {
        base.tick(model, echo);
        int altitude = (int)model.getAltitude();
        model.setBroadcast("EMERGENCY LANDING");
        if(altitude <= 2)
        {
            model.clearTargetVelocity();
            if(model.getShipVelocities().Length() < 1)
                model.doCharging();
        }
    }
    
    public override bool invalid(Shuttle model){ return model.getTotalChargePercent() > 0.3F; }
}