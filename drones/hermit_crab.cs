// == Hermit Crab Delivery Drone OS ==
const string version = "2.8";
const String spinning = "-\\|/";
const String config_default = 
    "[setup]\n;Prefix of this drone\nname_prefix=HMC\n;Name of the forward indicator light\nindicator_light=Interior Light\n\n[travel]\n;Style of travel (either forward or lateral)\nmove_style=forward\n;Top speed for this drone\nmax_speed=50\n;Yaw offset for this drone to face a given direction\nyaw_offset=0\n\n[altitude]\n;Whether this drone should ignore cockpit altitude readings\nignore_altitude=false\n;Minimum altitude this drone should maintain during travel\nminimum_altitude=50\nmaximum_altitude=75";
const int max_scan = 10000;

// Drone model
public static Drone model;
// Indicator light
public static IMyLightingBlock light;
public static IMyTextPanel screen;

// Config variables
private static MyIni config = new MyIni();
private static string namePrefix;
private static string lightName;
public static MoveStyle moveStyle;
public static double maxSpeed;
public static float yawOffset;
public static bool ignoreAltitude;
public static int minAltitude;
public static int maxAltitude;

readonly static Matrix directionMatrix = new Matrix(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
private static Dictionary<string, State> STATE_MAP = new Dictionary<String, State>();
private string state = "idle";

public static TravelDirection direction = TravelDirection.START;
public static int currentWaypoint = -1;
private static Vector3D destination;
private static int targetAltitude = -1;
private static int altitudeRange = 0;

private static int ticksRunning;
private bool hasClearedScreen;

public enum TravelDirection
{
    START,
    END
}
private static TravelDirection directionFromName(String nameIn)
{
    nameIn = nameIn.ToLower();
    foreach(TravelDirection style in Enum.GetValues(typeof(TravelDirection)))
        if(Enum.GetName(typeof(TravelDirection), style).ToLower() == nameIn)
            return style;
    return TravelDirection.START;
}
public enum MoveStyle
{
    FORWARD,
    LATERAL
}
private static MoveStyle styleFromName(String nameIn)
{
    nameIn = nameIn.ToLower();
    foreach(MoveStyle style in Enum.GetValues(typeof(MoveStyle)))
        if(Enum.GetName(typeof(MoveStyle), style).ToLower() == nameIn)
            return style;
    return MoveStyle.FORWARD;
}

public Program()
{
    Runtime.UpdateFrequency = UpdateFrequency.Update10;
    registerState("stop", new StateStop());
    registerState("stabilise", new StateStabilise());
    registerState("altitude", new StateAltitude());
    registerState("point", new StatePoint());
    registerState("move", new StateMove());
    registerState("lateral", new StateMoveLateral());
    registerState("land", new StateLand());
    registerState("land2", new StateLandPrecise());
    registerState("lowBat", new StateLowPower());
    
    if(Me.CustomData.Length == 0)
        Me.CustomData = config_default;
    loadData(Me.CustomData);
    Me.CubeGrid.CustomName = namePrefix;
    Me.CustomName = namePrefix+" Control";
    
    model = new Drone(Me, this);
    light = (IMyLightingBlock)GridTerminalSystem.GetBlockWithName(lightName);
    if(light != null)
    {
        light.Enabled = true;
        light.Color = new Color(255, 255, 255);
        light.BlinkIntervalSeconds = 0F;
    }
    
    List<IMyTextPanel> panels = new List<IMyTextPanel>();
    GridTerminalSystem.GetBlocksOfType<IMyTextPanel>(panels);
    foreach(var panel in panels)
        if(panel.CubeGrid == Me.CubeGrid)
        {
            panel.CustomName = namePrefix + " Status Screen";
            panel.Enabled = true;
            screen = panel;
            break;
        }
}

public void Save()
{
    Storage = String.Join(";", state, direction == TravelDirection.START ? 0 : 1, currentWaypoint, targetAltitude);
}

public void Main(string argument, UpdateType updateSource)
{
    hasClearedScreen = false;
    ++ticksRunning;
    Echo("Hermit Crab OS Version "+version+" "+getSpinning());
    
    if(argument.Length > 0)
        parseArg(argument);
    loadStorage(Storage);
    
    model.tick(Echo);
    if(!model.diagnostic(Echo))
    {
        setState("idle");
        model.setBroadcast("ERROR");
        return;
    }
    
    Echo("\nCurrent flight path: "+Enum.GetName(typeof(TravelDirection), direction)+" ("+currentWaypoint+")");
    MyWaypointInfo waypoint = getCurrentWaypoint();
    if(waypoint.IsEmpty())
        return;
    destination = waypoint.Coords;
    Echo("Destination: "+waypoint.Name);
    Echo("Coords: "+String.Join(", ", Math.Round(destination.X,2), Math.Round(destination.Y,2), Math.Round(destination.Z,2)));
    
    Echo("Available States:");
    foreach(var key in STATE_MAP.Keys)
        Echo((key==state ? " > " : " * ")+key);
    Echo("idle"==state ? " > idle" : " * idle");
    if(state != "idle")
    {
        echoToScreens("State: "+state);
        echoToScreens("Status Report:");
    }
    if(STATE_MAP.ContainsKey(state))
    {
        State currentState = STATE_MAP[state];
        currentState.tick(destination, model, echoToScreens);
        if(currentState.invalid(model))
            setState(currentState.exit(model));
    }
    else
        setState("idle");
    
    Save();
}

public void loadData(String customData)
{
    MyIniParseResult result;
    if(!config.TryParse(customData, out result))
    {
        Runtime.UpdateFrequency = UpdateFrequency.Once;
        throw new Exception(result.ToString());
    }
    
    namePrefix = config.Get("setup", "name_prefix").ToString("HMC");
    lightName = config.Get("setup", "indicator_light").ToString("Interior Light");
    
    moveStyle = styleFromName(config.Get("travel", "move_style").ToString("forward"));
    yawOffset = (float)config.Get("travel", "yaw_offset").ToDouble(0D);
    maxSpeed = config.Get("travel", "max_speed").ToDouble(50D);
    
    ignoreAltitude = config.Get("altitude", "ignore_altitude").ToBoolean(false);
    int altA = config.Get("altitude", "minimum_altitude").ToInt32(50);
    int altB = config.Get("altitude", "maximum_altitude").ToInt32(75);
    minAltitude = Math.Min(altA, altB);
    maxAltitude = Math.Max(altA, altB);
    altitudeRange = maxAltitude - minAltitude;
}

public void parseArg(String argument)
{
    String val = argument.ToLower();
    if(val == "reset")
        reset();
    else if(val == "remodel")
        model = new Drone(Me, this);
    else if(val.StartsWith("set_direction="))
        direction = directionFromName(val.Split('=')[1].ToLower());
    else if(val.StartsWith("set_index="))
        currentWaypoint = (int)clamp(int.Parse(val.Split('=')[1]), 0, model.getWaypoints().Count - 1);
    else if(val.StartsWith("set_state="))
        setState(val.Split('=')[1]);
    else if(val == "deliver" && model.getTotalChargePercent() > 0.8F)
    {
        currentWaypoint = direction == TravelDirection.START ? 1 : model.getWaypoints().Count - 2;
        direction = direction == TravelDirection.START ? TravelDirection.END : TravelDirection.START;
        setState("altitude");
    }
    
    Save();
}

public void loadStorage(String memory)
{
    reset();
    String[] lines = memory.ToLower().Split('|');
    int entries = lines.Length;
    
    // Operating data
    String[] data = lines[0].Split(';');
    if(data.Length == 4)
    {
        setState(data[0]);
        direction = int.Parse(data[1]) > 0 ? TravelDirection.END : TravelDirection.START;
        currentWaypoint = int.Parse(data[2]);
        targetAltitude = int.Parse(data[3]);
    }
    if(--entries <= 0) return;
}

public void echoToScreens(string text)
{
    Echo(text);
    if(screen != null && screen.IsFunctional && screen.Enabled)
        screen.WriteText((hasClearedScreen ? "\n" : "") + text, hasClearedScreen);
    hasClearedScreen = true;
}

private void reset()
{
    Save();
}

private static void registerState(String nameIn, State stateIn)
{
    if(nameIn == "" || stateIn == null)
        return;
    
    nameIn = nameIn.ToLower();
    STATE_MAP[nameIn] = stateIn;
}

// Exits the current state and moves to the first recognised one
private void setState(String statesIn)
{
    statesIn = statesIn.ToLower();
    string enterState = STATE_MAP.ContainsKey(statesIn) ? statesIn : "idle";
    
    if(enterState != state)
    {
        if(STATE_MAP.ContainsKey(state))
            STATE_MAP[state].exit(model);
        
        state = "";
        Save();
        
        state = enterState;
        if(STATE_MAP.ContainsKey(state))
        {
            State enteredState = STATE_MAP[state];
            enteredState.enter(destination, model);
            light.Color = enteredState.indicator;
        }
    }
    
    Save();
}

public MyWaypointInfo getCurrentWaypoint()
{
    return model.getWaypoint(currentWaypoint);
}

public static bool isFinalDestination()
{
    return isFinalDestination(currentWaypoint);
}

public static bool isFinalDestination(int index)
{
    return direction == TravelDirection.START ? index == 0 : index == model.getWaypoints().Count - 1;
}

public static void incrementWaypoint()
{
    currentWaypoint += direction == TravelDirection.START ? -1 : 1;
}

// #### UTILITY FUNCTIONS ####
public static char getSpinning()
{
    return spinning[ticksRunning % spinning.Length];
}

public static Vector3 getLocalFacing(IMyCubeBlock block, IMyRemoteControl remote)
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

public static bool isInRange(int val, int min, int max){ return val >= min && val <= max; }

public static int currentTargetAltitude()
{
    if(targetAltitude < 0)
        targetAltitude = minAltitude;
    
    /*
        If we are not in imminent danger of collision, allow target altitude to recede to configured altitude.
        This causes the target to drop when flying over lower terrain, reducing the need to descend.
    */
    if(targetAltitude > minAltitude && !model.collisionAlarm())
    {
        int current = targetAltitude - (ticksRunning%100 == 0 ? 1 : 0);
        int nextVal = (int)clamp(current, minAltitude, (int)model.getAltitude());
        int delta = nextVal - current;
        targetAltitude += Math.Sign(delta);
    }
    
    return targetAltitude;
}

public static bool altitudeWarning()
{
    int currentAltitude = (int)model.getAltitude();
    int target = currentTargetAltitude();
    return model.collisionAlarm() || !(ignoreAltitude || isInRange(currentAltitude, target, target + altitudeRange * 2));
}

public class Drone
{
    // Remote control
    private IMyRemoteControl remote;
    // Antenna
    private IMyRadioAntenna antenna;
    // Gyroscopes
    private List<IMyGyro> gyros = new List<IMyGyro>();
    // Terrain sensor
    private IMySensorBlock sensor;
    private IMyCameraBlock camera;
    // Docking connector
    private IMyShipConnector connector;
    // All batteries
    private List<IMyBatteryBlock> batteries = new List<IMyBatteryBlock>();
    // All thrusters
    private List<IMyThrust> thrusters = new List<IMyThrust>();
    private Dictionary<Base6Directions.Direction, List<IMyThrust>> thrustMap = new Dictionary<Base6Directions.Direction, List<IMyThrust>>();
    private PID pidX = new PID(), pidY = new PID(), pidZ = new PID();
    private Vector3D targetVelocity = new Vector3D(0, 0, 0);
    private Vector3D targetFacing = new Vector3D(0, 0, 0);
    
    private Vector3D forwRef;
    private Vector3D downRef;
    private Vector3D gravTarget;
    private double latestAltitude = 0;
    
    private String broadcastMessage = "";
    
    public Drone(IMyProgrammableBlock me, MyGridProgram program)
    {
        List<IMyRemoteControl> remotes = new List<IMyRemoteControl>();
        program.GridTerminalSystem.GetBlocksOfType<IMyRemoteControl>(remotes);
        foreach(var controller in remotes)
            if(controller.CubeGrid == me.CubeGrid && controller.IsFunctional)
            {
                controller.CustomName = namePrefix+" Remote";
                controller.DampenersOverride = true;
                remote = controller;
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
        if(remote != null)
        {
            List<IMyShipConnector> connectors = new List<IMyShipConnector>();
            program.GridTerminalSystem.GetBlocksOfType<IMyShipConnector>(connectors);
            foreach(var connect in connectors)
                if(connect.CubeGrid == me.CubeGrid && getLocalFacing(connect, remote) == directionMatrix.Up)
                {
                    connect.CustomName = namePrefix+" Parking Connector";
                    connect.Enabled = true;
                    connect.ShowInToolbarConfig = false;
                    
                    connector = connect;
                }
            
            List<IMyThrust> thrusters2 = new List<IMyThrust>();
            program.GridTerminalSystem.GetBlocksOfType<IMyThrust>(thrusters2);
            foreach(var thruster in thrusters2)
            {
                if(thruster.CubeGrid == me.CubeGrid)
                {
                    thruster.ShowInTerminal = false;
                    thruster.ShowInToolbarConfig = false;
                    
                    Vector3 thrustVec = getLocalFacing(thruster, remote);
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
                if(cam.CubeGrid == me.CubeGrid && getLocalFacing(cam, remote) == directionMatrix.Up)
                {
                    cam.CustomName = namePrefix+" Altitude Sensor";
                    cam.Enabled = true;
                    cam.EnableRaycast = true;
                    camera = cam;
                    break;
                }
        }
        
        List<IMySensorBlock> sensors = new List<IMySensorBlock>();
        program.GridTerminalSystem.GetBlocksOfType<IMySensorBlock>(sensors);
        foreach(var sense in sensors)
            if(sense.CubeGrid == me.CubeGrid && sense.IsFunctional)
            {
                sense.CustomName = namePrefix+" Collision Sensor";
                sense.Enabled = true;
                
                Vector3 faceVec = getLocalFacing(sense, remote);
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
        
        if(remote.IsUnderControl)
        {
            disableControl();
            return;
        }
        else if(remote == null || !remote.IsFunctional)
        {
            disableControl();
            setBroadcast("IN DISTRESS");
            return;
        }
        
        updateOrientationRef();
        updateLevelling();
        
        // Total mass of ship (kg)
        double mass = remote.CalculateShipMass().TotalMass;
        
        // Downward gravity relative to ship (m/s)
        double grav = toLocal(getGravity()).Length();
        
        // Velocity relative to ship (m/s)
        Vector3D velocity = toLocal(remote.GetShipVelocities().LinearVelocity);
        
        // Target velocity relative to ship (m/s)
        Vector3D targetVel = toLocal(targetVelocity);
        
        handleThrust(targetVel.X, velocity.X, pidX, mass, getThrusters(Base6Directions.Direction.Right), getThrusters(Base6Directions.Direction.Left));
        handleThrust(targetVel.Y, velocity.Y, pidY, mass, getThrusters(Base6Directions.Direction.Up), Math.Abs(grav)*mass);
        handleThrust(targetVel.Z, velocity.Z, pidZ, mass, getThrusters(Base6Directions.Direction.Backward), getThrusters(Base6Directions.Direction.Forward));
    }
    
    private void disableControl()
    {
        resetGyros();
        resetThrusters(thrusters);
    }
    
    // Runs a diagnostic of all vital components and returns true if they all pass
    public bool diagnostic(Action<string> echo)
    {
        bool pass = true;
        if(remote == null || !remote.IsFunctional)
        {
            echo(" X Missing remote control");
            pass = false;
        }
        else
        {
            List<MyWaypointInfo> waypoints = new List<MyWaypointInfo>();
            remote.GetWaypointInfo(waypoints);
            
            if(waypoints.Count < 2)
            {
                echo(" X Not enough waypoints registered");
                pass = false;
            }
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
        
        if(connector == null || !connector.IsFunctional)
        {
            echo(" X No functional parking connector");
            pass = false;
        }
        
        if(camera == null || !camera.IsFunctional)
            echo(" ? Missing altitude camera");
        
        if(antenna == null || !antenna.IsFunctional)
            echo(" ? Missing antenna");
        
        if(light == null)
            echo(" ? Missing indicator light named "+lightName);
        
        return pass;
    }
    
    public List<MyWaypointInfo> getWaypoints()
    {
        List<MyWaypointInfo> waypoints = new List<MyWaypointInfo>();
        remote.GetWaypointInfo(waypoints);
        return waypoints;
    }
    
    public MyWaypointInfo getWaypoint(int index)
    {
        List<MyWaypointInfo> waypoints = getWaypoints();
        index = (int)clamp(index, 0, waypoints.Count - 1);
        
        return waypoints[index];
    }
    
    public IMyRemoteControl getRemote(){ return remote; }
    public Vector3D getGravity(){ return remote == null ? new Vector3D(0, 0, 0) : remote.GetNaturalGravity(); }
    public Vector3D getWorldPosition(){ return connector.GetPosition(); }
    
    public Vector3D toLocal(Vector3D vectorIn){ return Vector3D.TransformNormal(vectorIn, MatrixD.Transpose(remote.WorldMatrix)); }
    public Vector3D toWorld(Vector3D vectorIn){ return Vector3D.TransformNormal(vectorIn, remote.WorldMatrix); }
    
    public void setSensor(bool onOff){ if(sensor != null) sensor.Enabled = onOff; }
    
    public void setBroadcast(String messageIn)
    {
        if(messageIn.Length > 0 && !messageIn.StartsWith(namePrefix))
            messageIn = namePrefix + " " + messageIn;
        broadcastMessage = messageIn;
    }
    
    public double getAltitude()
    {
        if(camera == null || !(camera.Enabled && camera.IsFunctional))
        {
            // If camera is absent or non-functional, use cockpit reading
            // WARNING! This reading is drawn from voxels at worldgen and can be very inaccurate
            double altSea = 0D;
            double altVox = 0D;
            remote.TryGetPlanetElevation(MyPlanetElevation.Sealevel, out altSea);
            remote.TryGetPlanetElevation(MyPlanetElevation.Surface, out altVox);
            return Math.Min(altSea, altVox);
        }
        
        camera.EnableRaycast = true;
        int scanDist = Math.Max(max_scan, minAltitude);
        if(camera.CanScan(scanDist))
        {
            MyDetectedEntityInfo scan = camera.Raycast(scanDist);
            if(scan.IsEmpty())
                latestAltitude = scanDist + 1;
            else if(scan.HitPosition.HasValue)
                latestAltitude = (camera.GetPosition() - scan.HitPosition.Value).Length();
        }
        return latestAltitude;
    }
    
    public bool collisionAlarm(){ return sensor != null && sensor.Enabled && sensor.IsActive; }
    
    public IMyShipConnector getConnector(){ return connector; }
    public void setConnector(bool on){ if(connector != null) connector.Enabled = on; }
    public bool isConnected(){ return (connector == null || !connector.IsFunctional) ? false : connector.Status == MyShipConnectorStatus.Connected; }
    public MyShipConnectorStatus getConnectorStatus(){ return connector == null || !connector.IsFunctional ? MyShipConnectorStatus.Unconnected : connector.Status; }
    public void toggleConnect(){ if(connector != null) connector.ToggleConnect(); }
    
    public void setTargetFacing(Vector3D face){ targetFacing = face; }
    public void clearTargetFacing(){ setTargetFacing(new Vector3D(0, 0, 0)); }
    
    // Sets gyroscopes to maintain gravity level on Pitch and Roll axises
    private void updateLevelling()
    {
        resetGyros();
        double error = currentFacingError();
        double currentSpin = toLocal(getShipRotations()).Y;
        double spin = targetFacing.Length() > 0 && Math.Abs(error) > 1 && currentSpin < 5 ? -MathHelper.Clamp(error, -10, 10) * 0.1D : 0D;
        
        foreach(var gyro in gyros)
        {
            if(gyro == null || !gyro.Enabled || !gyro.IsFunctional) continue;
            gyro.GyroOverride = true;
            Matrix localOrientation;
            gyro.Orientation.GetMatrix(out localOrientation);
            
            if(Vector3D.TransformNormal(Vector3D.Up, gyro.WorldMatrix) == toWorld(Vector3D.Up))
                gyro.Yaw = (float)spin;
            
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
    
    public float currentFacingError()
    {
        return getFacingError(targetFacing);
    }
    
    public float getFacingError(Vector3D vector)
    {
        Vector3D forwLocal = toLocal(vector);
        double yawV = Math.Atan2(forwLocal.Z, forwLocal.X);
        double yawL = Math.Atan2(forwRef.Z, forwRef.X);
        return (float)MathHelper.ToDegrees(yawL - yawV) + yawOffset;
    }

    // Updates the orientation values used for navigation and levelling
    private void updateOrientationRef()
    {
        Matrix shipOrientation;
        remote.Orientation.GetMatrix(out shipOrientation);
        
        var quatPitch = Quaternion.CreateFromAxisAngle(shipOrientation.Left, 0F);
        var quatRoll = Quaternion.CreateFromAxisAngle(shipOrientation.Backward, 0F);
        
        forwRef = Vector3D.Transform(shipOrientation.Forward, quatPitch * quatRoll);
        downRef = Vector3D.Transform(shipOrientation.Down, quatPitch * quatRoll);
        gravTarget = remote.GetNaturalGravity();
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
    
    public Vector3D getShipVelocities(){ return remote.GetShipVelocities().LinearVelocity; }
    public Vector3D getShipRotations(){ return remote.GetShipVelocities().AngularVelocity; }
    
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
            thruster.Enabled = true;
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
        int tallyA = 0;
        float avgThrustA = 0F;
        foreach(var thruster in groupA)
            if(thruster != null && thruster.IsFunctional)
            {
                tallyA++;
                avgThrustA += thruster.MaxThrust;
            }
        avgThrustA /= tallyA;
        
        int tallyB = 0;
        float avgThrustB = 0F;
        foreach(var thruster in groupB)
            if(thruster != null && thruster.IsFunctional)
            {
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

public abstract class State
{
    private Controls[] affectedComponents = new Controls[0];
    public Color indicator = new Color(255, 255, 255);
    
    protected State(Controls[] affectedIn) : this(new Color(255, 255, 255), affectedIn){ }
    protected State(Color colorIn, Controls[] affectedIn)
    {
        indicator = colorIn;
        affectedComponents = affectedIn;
    }
    
    // Performs any basic initial settings this state needs
    public virtual void enter(Vector3D destination, Drone model){ }
    // Performs the functions of this state
    public abstract void tick(Vector3D destination, Drone model, Action<string> echo);
    // Returns true if a state change should occur, ie. this state's work is finished and it can safely stop
    public abstract bool invalid(Drone model);
    // Resets any changes caused by this state and returns the state names that should be moved into
    public virtual String exit(Drone model)
    {
        resetModel(model);
        onExit(model);
        return exitState(model);
    }
    public virtual void onExit(Drone model){ }
    public virtual String exitState(Drone model){ return "idle"; }
    
    protected virtual void resetModel(Drone model)
    {
        foreach(var component in affectedComponents)
            switch(component)
            {
                case Controls.THRUSTERS: model.clearTargetVelocity(); break;
                case Controls.GYROS: model.clearTargetFacing(); break;
                case Controls.CONNECTOR: model.setConnector(true); if(model.isConnected()) model.toggleConnect(); break;
                case Controls.BATTERIES: model.resetBatteries(); break;
                case Controls.SENSOR: model.setSensor(true); break;
                case Controls.LIGHT: light.Enabled = true; break;
                case Controls.ANTENNA: model.setBroadcast(null); break;
            }
    }
    
    public enum Controls
    {
        THRUSTERS,
        GYROS,
        CONNECTOR,
        BATTERIES,
        SENSOR,
        LIGHT,
        ANTENNA
    }
}

public class StateStop : State
{
    public StateStop() : base(new Controls[]{Controls.BATTERIES, Controls.SENSOR, Controls.LIGHT, Controls.CONNECTOR}){ }
    
    public override void tick(Vector3D destination, Drone model, Action<string> echo)
    {
        float charge = model.getTotalChargePercent();
        echo("   > Charge: "+Math.Round(charge, 2)*100+"%");
        echo("   > "+(charge >= 0.8F ? "Launch ready" : "Recharging"));
        model.doCharging();
        model.setSensor(false);
        targetAltitude = -1;
    }
    
    public override bool invalid(Drone model){ return model.getAltitude() > 5 && !model.isConnected(); }
    public override String exitState(Drone model){ return "land"; }
}

public class StateStabilise : State
{
    public StateStabilise() : base(new Color(255, 0, 255), new Controls[0]){ }
    
    public override void tick(Vector3D destination, Drone model, Action<string> echo){ }
    public override bool invalid(Drone model){ return model.getShipRotations().Length() < 0.5D; }
    public override String exitState(Drone model){ return "point"; }
}

public class StateAltitude : State
{
    private bool initialCollision;
    public StateAltitude() : base(new Color(255, 0, 0), new Controls[]{Controls.THRUSTERS}){ }
    
    public override void enter(Vector3D destination, Drone model){ initialCollision = model.collisionAlarm(); }
    
    public override void tick(Vector3D destination, Drone model, Action<string> echo)
    {
        double altitude = model.getAltitude();
        int targetAlt = currentTargetAltitude();
        bool collision = model.collisionAlarm();
        
        if(model.isConnected())
            model.toggleConnect();
        
        int offset = altitudeRange / 4;
        bool goodAlt = altitude >= (targetAlt + offset) && altitude <= (targetAlt + altitudeRange - offset);
        if(collision)
        {
            // If altitude is too high but the collision alarm is going off, increase target to current altitude
            if(altitude >= (targetAltitude + altitudeRange))
                targetAltitude = (int)altitude;
            // If altitude is within range but the collision alarm is going off, increase target altitude.
            else if(goodAlt)
                targetAltitude += 1;
        }
        
        int target = targetAltitude + (altitudeRange / 2);
        int error = target - (int)altitude;
        
        echo("   > Altitude: "+targetAltitude+" << "+Math.Round(altitude, 2)+" >> "+(targetAltitude + altitudeRange));
        if(collision)
            echo("     > Collision Alarm!");
        
        if(goodAlt && !collision)
            model.clearTargetVelocity();
        else
        {
            int speed = (int)Math.Round((double)error / 10, 0);
            if(Math.Abs(speed) < 1)
                speed = Math.Sign(speed);
            
            Vector3D gravity = model.getGravity();
            gravity.Normalize();
            gravity = Vector3D.Multiply(gravity, -1);
            
            model.setTargetVelocity(Vector3D.Multiply(gravity, speed));
        }
    }
    
    public override bool invalid(Drone model)
    {
        double altitude = model.getAltitude();
        int targetAlt = currentTargetAltitude();
        int offset = altitudeRange / 4;
        return model.getShipVelocities().Length() < 1 && !model.collisionAlarm() && altitude >= (targetAlt + offset) && altitude <= (targetAlt + altitudeRange - offset);
    }
    public override String exitState(Drone model){ return "point"; }
}

public class StatePoint : State
{
    private Vector3D dest;
    public StatePoint() : base(new Color(255, 165, 0), new Controls[]{Controls.GYROS}){ }
    
    public override void tick(Vector3D destination, Drone model, Action<string> echo)
    {
        if(moveStyle != MoveStyle.FORWARD)
            return;
        
        dest = destination;
        Vector3D direction = (dest - model.getWorldPosition());
        direction.Normalize();
        model.setTargetFacing(direction);
        
        echo(" > Error: "+Math.Round(model.currentFacingError(), 2));
    }
    
    public override bool invalid(Drone model)
    {
        return moveStyle != MoveStyle.FORWARD || Math.Abs(model.currentFacingError()) < 1 || getLateralDirection(model).Length() < 15D;
    }
    public override String exitState(Drone model){ return "move"; }
    
    protected virtual Vector3D getLateralDirection(Drone model)
    {
        Vector3D world = model.toLocal(dest - model.getWorldPosition());
        world.Y = 0;
        return model.toWorld(world);
    }
}

public class StateMove : State
{
    protected Vector3D dest;
    private double cutoffDist;
    protected String exitCause = "";
    protected bool allowAltitude = true;
    
    public StateMove() : this(200D, new Color(0, 255, 255), new Controls[]{Controls.THRUSTERS}){ }
    public StateMove(double distIn, Color colorIn) : this(distIn, colorIn, new Controls[]{Controls.THRUSTERS}){ }
    public StateMove(Color colorIn) : this(20D, colorIn, new Controls[]{Controls.THRUSTERS}){ }
    public StateMove(Color colorIn, Controls[] controlsIn) : this(20D, colorIn, controlsIn){ }
    protected StateMove(double distIn, Color colorIn, Controls[] controlsIn) : base(colorIn, controlsIn)
    {
        cutoffDist = distIn;
    }
    
    public override void tick(Vector3D destination, Drone model, Action<string> echo)
    {
        dest = destination;
        Vector3D direction = getLateralDirection(model);
        double speedLimit = getSpeedLimit(destination, model);
        double altitude = model.getAltitude();
        int altTarget = currentTargetAltitude();
        
        Vector3D velocities = model.toLocal(model.getShipVelocities());
        echo("   > Distance: "+Math.Round(direction.Length(), 2));
        if(!ignoreAltitude && allowAltitude)
            echo("   > Altitude: "+Math.Round(altitude, 0)+" / ["+altTarget+"-"+(altTarget+altitudeRange)+"]");
        echo("   > Max Velocity: "+Math.Round(speedLimit, 2)+"m/s");
        echo("   > Current: "+Math.Round(velocities.Length(), 2)+"m/s");
        
        if(direction.Length() > cutoffDist && velocities.Length() < speedLimit)
        {
            direction.Normalize();
            
            if(allowAltitude && !ignoreAltitude)
            {
                altTarget += (altitudeRange / 2);
                double altError = altTarget - altitude;
                
                if(Math.Abs(altError) > (altitudeRange / 8))
                {
                    Vector3D gravity = model.getGravity();
                    gravity.Normalize();
                    gravity = Vector3D.Multiply(gravity, -1);
                    
                    direction += Vector3D.Multiply(gravity, Math.Sign(altError) * 0.1D);
                }
            }
            
            model.setTargetVelocity(Vector3D.Multiply(direction, speedLimit));
        }
        else
            model.clearTargetVelocity();
    }
    
    public override bool invalid(Drone model)
    {
        if(model.getTotalChargePercent() < 0.2F)
        {
            exitCause = "lowBat";
            return true;
        }
        else if(altitudeWarning())
        {
            exitCause = "altitude";
            return true;
        }
        else if(model.getShipVelocities().Length() < 1 && getLateralDirection(model).Length() <= cutoffDist)
        {
            exitCause = "lateral";
            return true;
        }
        else if(moveStyle == MoveStyle.FORWARD && Math.Abs(model.getFacingError(destination - model.getWorldPosition())) > 5D)
        {
            exitCause = "point";
            return true;
        }
        
        return false;
    }
    
    public override String exitState(Drone model){ return exitCause; }
    
    protected virtual Vector3D getLateralDirection(Drone model)
    {
        Vector3D world = model.toLocal(dest - model.getWorldPosition());
        world.Y = 0;
        return model.toWorld(world);
    }
    
    protected virtual double getVerticalDistance(Drone model)
    {
        Vector3D world = dest - model.getWorldPosition();
        world = model.toLocal(world);
        return Math.Abs(world.Y);
    }
    
    protected virtual double getSpeedLimit(Vector3D destination, Drone model){ return Math.Min(maxSpeed, getLateralDirection(model).Length() / 10); }
}

public class StateMoveLateral : StateMove
{
    private bool incOnExit = false;
    public StateMoveLateral() : base(0.5D, new Color(0, 255, 0)){ allowAltitude = false; }
    
    protected override double getSpeedLimit(Vector3D destination, Drone model){ return Math.Max(getLateralDirection(model).Length() / 10, 0.05D); }
    
    public override void onExit(Drone model)
    {
        if(incOnExit)
        {
            incOnExit = false;
            incrementWaypoint();
        }
    }
    
    public override bool invalid(Drone model)
    {
        double direction = getLateralDirection(model).Length();
        if(altitudeWarning())
        {
            exitCause = "altitude";
            return true;
        }
        else if(direction > 200)
        {
            exitCause = "move";
            return true;
        }
        else if(model.getShipVelocities().Length() < 1)
            if(direction < 10 && !isFinalDestination())
            {
                incOnExit = true;
                exitCause = "point";
                return true;
            }
            else if(direction < 0.5D)
            {
                exitCause = "land";
                return true;
            }
        
        return false;
    }
}

public class StateLand : StateMove
{
    private bool prevCollision = false;
    
    public StateLand() : this(new Color(0, 255, 255)){ }
    protected StateLand(Color colorIn) : base(colorIn, new Controls[]{Controls.THRUSTERS, Controls.ANTENNA}){ }
    
    public override void tick(Vector3D destination, Drone model, Action<string> echo)
    {
        dest = destination;
        Vector3D deviation = getLateralDirection(model);
        double distVert = (destination - model.getWorldPosition()).Length() - deviation.Length();
        int altitude = (int)model.getAltitude();
        echo("   > Altitude: "+altitude);
        echo("   > Distance: "+Math.Round(distVert,2));
        echo("   > Deviation: "+Math.Round(deviation.Length(),2));
        model.setBroadcast("Landing "+(int)distVert+"m");
        
        if((ignoreAltitude ? distVert > 0 : altitude > 2) && !model.isConnected())
        {
            Vector3D direction = (dest - model.getWorldPosition());
            direction.Normalize();
            
            direction = model.toLocal(direction);
            direction.Y = Math.Min(-1, direction.Y);
            direction = model.toWorld(direction);
            
            double speed = Math.Min(distVert, altitude);
            model.setTargetVelocity(Vector3D.Multiply(direction, model.collisionAlarm() ? Math.Min(2D, speed) : 15D));
        }
        else
            model.clearTargetVelocity();
        
        if(model.getConnectorStatus() == MyShipConnectorStatus.Connectable)
            model.toggleConnect();
    }
    
    public override bool invalid(Drone model)
    {
        bool landed = ignoreAltitude ? (dest - model.getWorldPosition()).Length() == 0 : model.getAltitude() <= 2;
        if(landed || model.isConnected())
        {
            exitCause = "stop";
            return true;
        }
        else if(getLateralDirection(model).Length() > 5 && prevCollision != model.collisionAlarm())
        {
            exitCause = "land2";
            return true;
        }
        prevCollision = model.collisionAlarm();
        return false;
    }
}

public class StateLandPrecise : StateLand
{
    public StateLandPrecise() : base(new Color(0, 0, 255)){ }
    
    public override void tick(Vector3D destination, Drone model, Action<string> echo)
    {
        dest = destination;
        Vector3D direction = getLateralDirection(model);
        Vector3D velocities = model.toLocal(model.getShipVelocities());
        
        echo("   > Distance: "+Math.Round(direction.Length(),2));
        echo("   > Velocities: "+String.Join(", ", Math.Abs(Math.Round(velocities.X,2)), Math.Abs(Math.Round(velocities.Z,2))));
        
        if(direction.Length() > 0.5D)
        {
            direction.Normalize();
            model.setTargetVelocity(direction);
        }
        else
            model.clearTargetVelocity();
    }
    
    public override bool invalid(Drone model){ return getLateralDirection(model).Length() < 0.5D; }
    public override String exitState(Drone model){ return "land"; }
}

public class StateLowPower : State
{
    public StateLowPower() : base(new Color(0,0,0), new Controls[]{Controls.THRUSTERS, Controls.ANTENNA}){ }
    
    public override void tick(Vector3D destination, Drone model, Action<string> echo)
    {
        int altitude = (int)model.getAltitude();
        model.setBroadcast("EMERGENCY LANDING");
        if(altitude > 3)
        {
            Vector3D gravity = model.getGravity();
            gravity.Normalize();
            model.setTargetVelocity(Vector3D.Multiply(gravity, Math.Max(1, altitude / 15)));
        }
        else
        {
            model.clearTargetVelocity();
            if(model.getShipVelocities().Length() < 1)
                model.doCharging();
        }
    }
    
    public override bool invalid(Drone model){ return model.getTotalChargePercent() > 0.5F; }
    public override String exitState(Drone model){ return "altitude"; }
}