// == Termite Miner OS ==
const string version = "1.75";
const string config_default = 
    "[setup]\n;Number of clockwise 90 degree turns to apply\n;to the drone's forward mining direction\nrotation=0\n\n[mining_area]\n;Size of the drone's mining area\ngrid_size=10\n;Size of the drone's mining footprint\ncell_size=5\n;The maximum mining depth of each cell\nbore_depth=50\n;Grid search pattern (simple, corner_spiral, spiral, or offset)\nstyle=spiral";

private List<IMyTextPanel> screensAll = new List<IMyTextPanel>();
private List<IMyTextPanel> screensInfo = new List<IMyTextPanel>();
private List<IMyTextPanel> screensState = new List<IMyTextPanel>();
private bool hasClearedScreen = false;

// Config values
private static MyIni config = new MyIni();
private static GridStyle gridStyle;
private static int rotation;
private static int gridSize;
private static double stepDist;
public static int boreDepth;

private static bool hasHomePos = false;
private static Vector3D homePos = new Vector3D(0,0,0);

private static bool hasMinePos = false;
private static Vector3D minePos = new Vector3D(0,0,0);
private static Vector3D moveForw = new Vector3D(0, 0, 1);
private static Vector3D moveRight = new Vector3D(-1, 0, 1);
private static int index=0;
private static bool incFlag = false;

readonly static Matrix directionMatrix = new Matrix(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
static Dictionary<string, State> STATE_MAP = new Dictionary<String, State>();
private string state = "idle";
private int ticksInvalid = 0;

public Drone model;
private static MyItemType[] ores = new MyItemType[]
    {
        MyItemType.MakeOre("Iron"),
        MyItemType.MakeOre("Silicon"),
        MyItemType.MakeOre("Nickel"),
        MyItemType.MakeOre("Cobalt"),
        MyItemType.MakeOre("Silver"),
        MyItemType.MakeOre("Platinum"),
        MyItemType.MakeOre("Magnesium"),
        MyItemType.MakeOre("Ice"),
        MyItemType.MakeOre("Uranium")
    };
private static MyItemType stone = MyItemType.MakeOre("Stone");

private int ticksRunning;
const String spinning = "-\\|/";

private enum GridStyle
    {
        SIMPLE,    // Basic rows & columns, starting from mining position
        CORNER_SPIRAL,    // Outward spiral, starting from mining position
        SPIRAL,    // Outward spiral, centred on mining position
        OFFSET    // Simple, centred on mining position
    }
private static GridStyle styleFromName(String nameIn)
{
    nameIn = nameIn.ToLower();
    foreach(GridStyle style in Enum.GetValues(typeof(GridStyle)))
        if(Enum.GetName(typeof(GridStyle), style).ToLower() == nameIn)
            return style;
    return GridStyle.SIMPLE;
}

public Program()
{
    Runtime.UpdateFrequency = UpdateFrequency.Update10;
    registerState("detach", new StateDetach());
    registerState("travel_mine", new StateGoMine());
    registerState("dig", new StateDig());
    registerState("extract", new StateExtract());
    registerState("travel_home", new StateGoHome());
    registerState("dock", new StateDock());
    registerState("deposit", new StateDeposit());
    
    Me.CustomName = "TMT Drone Control";
    if(Me.CustomData.Length == 0)
        Me.CustomData = config_default;
    loadData(Me.CustomData);
    model = new Drone(Me, this);
    
    List<IMyTextPanel> panels = new List<IMyTextPanel>();
    GridTerminalSystem.GetBlocksOfType<IMyTextPanel>(panels);
    foreach(var panel in panels)
        if(panel.CubeGrid == Me.CubeGrid)
        {
            panel.CustomName = "TMT Info Screen "+screensAll.Count;
            string data = panel.CustomData.ToLower();
            if(data == "state")
                screensState.Add(panel);
            else
                screensInfo.Add(panel);
            screensAll.Add(panel);
        }
}

public void Save()
{
    Storage = string.Join(";", index, state);
    Storage += "|" + string.Join(";", (hasHomePos ? 1 : 0), homePos.X, homePos.Y, homePos.Z);
    Storage += "|" + string.Join(";", (hasMinePos ? 1 : 0), minePos.X, minePos.Y, minePos.Z);
    Storage += "|" + string.Join(";", moveForw.X, moveForw.Y, moveForw.Z);
    Storage += "|" + string.Join(";", moveRight.X, moveRight.Y, moveRight.Z);
}

public void Main(string argument, UpdateType updateSource)
{
    hasClearedScreen = false;
    ++ticksRunning;
    echoToScreens("Termite OS Version "+version+" "+getSpinning()+"\n");
    echoToScreens("Power: "+Math.Round(model.getTotalChargePercent(),2)*100+"%");
    echoToScreens("Borehole Index: "+index+(incFlag ? "+" : ""));
    
    if(argument.Length > 0)
        parseArg(argument);
    loadStorage(Storage);
    
    model.tick();
    if(!hasHomePos && model.isConnected())
        bindHomeFromConnector(model);
    if(!model.runDiagnostic(echoToScreens) || model.isUnderControl())
        setState(new String[]{"idle"});
    
    echoToScreens("Home position:", screensInfo);
    Vector3D home = getHomePosition();
    echoToScreens("  "+vectorToString(home));
    
    Vector3D mine = getMinePosition();
    if((home - mine).Length() > stepDist)
    {
        echoToScreens("Mining position:", screensInfo);
        echoToScreens("  "+vectorToString(mine));
    }
    
    echoToScreens("Mining direction:", screensInfo);
    Vector3D direct = Vector3D.Multiply(getDirection(rotation), stepDist);
    echoToScreens("  "+vectorToString(direct));
    
    echoToScreens("Current job ("+gridSize+"x"+gridSize+"x"+stepDist+"m, "+boreDepth+"m, "+Enum.GetName(typeof(GridStyle), gridStyle).ToLower()+")", screensInfo);
    Vector3D digSite = getMiningPosition();
    echoToScreens("  "+vectorToString(digSite), screensInfo);
    echoToScreens("  Travel Distance "+Math.Round((home-digSite).Length(), 2), screensInfo);
    Echo("");
    
    hasClearedScreen = false;
    echoToStateScreens("State: "+state);
    if(ticksInvalid > 0)
        echoToStateScreens(" "+getSpinning()+" Changing State "+ticksInvalid);
    
    if(STATE_MAP.ContainsKey(state))
    {
        State currentState = STATE_MAP[state];
        currentState.tick(index, rotation, model, echoToStateScreens);
        if(currentState.invalid(model) && ticksInvalid++ >= 10)
        {
            setState(currentState.exit(model));
            ticksInvalid = 0;
        }
    }
    else
        setState(new String[]{"idle"});
    
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
    
    rotation = config.Get("setup", "rotation").ToInt32(0) % 4;
    
    gridSize = config.Get("mining_area", "grid_size").ToInt32(10);
    stepDist = config.Get("mining_area", "cell_size").ToDouble(5D);
    boreDepth = config.Get("mining_area", "bore_depth").ToInt32(50);
    gridStyle = styleFromName(config.Get("mining_area","style").ToString("spiral"));
}

public void parseArg(String argument)
{
    String val = argument.ToLower();
    if(val == "reset")
        reset();
    else if(val == "remodel")
        model = new Drone(Me, this);
    else if(val.StartsWith("set_index="))
        index = int.Parse(val.Split('=')[1]);
    else if(val == "set_home")
        bindHomeFromConnector(model);
    else if(val == "set_mine")
        setMinePosition(model);
    else if(val.StartsWith("set_state="))
        setState(new String[]{val.Split('=')[1]});
    
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
        index = int.Parse(data[0]);
        setState(new String[]{data[1]});
    }
    if(--entries <= 0) return;
    
    // Home position
    data = lines[1].Split(';');
    if(data.Length == 4)
    {
        hasHomePos = int.Parse(data[0]) > 0;
        if(hasHomePos)
        {
            double x = double.Parse(data[1]);
            double y = double.Parse(data[2]);
            double z = double.Parse(data[3]);
            homePos = new Vector3D(x, y, z);
        }
    }
    if(--entries <= 0) return;
    
    // Mine position
    data = lines[2].Split(';');
    if(data.Length == 4)
    {
        hasMinePos = int.Parse(data[0]) > 0;
        if(hasMinePos)
        {
            double x = double.Parse(data[1]);
            double y = double.Parse(data[2]);
            double z = double.Parse(data[3]);
            minePos = new Vector3D(x, y, z);
        }
    }
    if(--entries <= 0) return;
    
    // Forward vector
    data = lines[3].Split(';');
    if(data.Length == 3)
    {
        double x = double.Parse(data[0]);
        double y = double.Parse(data[1]);
        double z = double.Parse(data[2]);
        moveForw = new Vector3D(x, y, z);
    }
    if(--entries <= 0) return;
    
    // Right vector
    data = lines[4].Split(';');
    if(data.Length == 3)
    {
        double x = double.Parse(data[0]);
        double y = double.Parse(data[1]);
        double z = double.Parse(data[2]);
        moveRight = new Vector3D(x, y, z);
    }
    if(--entries <= 0) return;
}

public void reset()
{
    index = 0;
    rotation = 0;
    hasHomePos = false;
    homePos = new Vector3D(0,0,0);
    hasMinePos = false;
    minePos = new Vector3D(0,0,0);
    Save();
}

public char getSpinning()
{
    return spinning[this.ticksRunning % spinning.Length];
}

private static void registerState(String nameIn, State stateIn)
{
    if(nameIn == "" || stateIn == null)
        return;
    
    nameIn = nameIn.ToLower();
    STATE_MAP[nameIn] = stateIn;
}

// Exits the current state and moves to the first recognised one
private void setState(String[] statesIn)
{
    bool found = false;
    string enterState = state;
    foreach(String stateName in statesIn)
        if(STATE_MAP.ContainsKey(stateName.ToLower()))
        {
            enterState = stateName.ToLower();
            found = true;
            break;
        }
    if(!found)
        enterState = "idle";
    
    if(enterState != state)
    {
        if(STATE_MAP.ContainsKey(state))
            STATE_MAP[state].exit(model);
        
        state = "";
        Save();
        
        state = enterState;
        if(STATE_MAP.ContainsKey(state))
            STATE_MAP[state].enter(index, rotation, model);
    }
    
    Save();
}

public void echoToScreens(string text){ echoToScreens(text, screensAll); }
public void echoToStateScreens(string text){ echoToScreens(text, screensState); }
public void echoToScreens(string text, List<IMyTextPanel> screens)
{
    Echo(text);
    foreach(var screen in screens)
        if(screen != null && screen.IsFunctional && screen.Enabled)
            screen.WriteText((hasClearedScreen ? "\n" : "") + text, hasClearedScreen);
    hasClearedScreen = true;
}

public static String vectorToString(Vector3D vector){ return String.Join(", ", Math.Round(vector.X,2), Math.Round(vector.Y,2), Math.Round(vector.Z,2)); }

public static void increment()
{
    index++;
    incFlag = false;
}

public static bool hasHomePosition(){ return hasHomePos; }
public static Vector3D getHomePosition(){ return homePos; }
private static void setHomePosition(Vector3D vector)
{
    hasHomePos = true;
    homePos = vector;
}
public static void bindHomeFromConnector(Drone model)
{
    setHomePosition(model.getWorldPosition());
    if(!hasMinePosition())
        setMinePosition(model);
}

public static bool hasMinePosition(){ return hasMinePos; }
public static Vector3D getMinePosition(){ return minePos; }
private static void setMinePosition(Drone model)
{
    hasMinePos = true;
    minePos = model.getWorldPosition();
    moveForw = Vector3D.TransformNormal(Vector3.Forward, model.getRemote().WorldMatrix);
    moveForw.Normalize();
    moveRight = Vector3D.TransformNormal(Vector3.Right, model.getRemote().WorldMatrix);
    moveRight.Normalize();
    rotation = 0;
}

public static Vector3D getDirection(int turns)
{
    Vector3D forward = moveForw;
    Vector3D right = moveRight;
    
    while(turns-- > 0)
    {
        Vector3D temp = Vector3D.Multiply(forward, -1);
        forward = right;
        right = temp;
    }
    return forward;
}

public static Vector3D getMiningPosition()
{
    Vector3D forward = getDirection(rotation);
    forward.Normalize();
    
    int row = 0;
    int col = 0;
    switch(gridStyle)
    {
        case GridStyle.CORNER_SPIRAL:
            var layer = Math.Floor(Math.Sqrt(index));
            var indexInLayer = index - layer * (layer + 1);
            row = (int)(layer + Math.Min(indexInLayer, 0));
            col = (int)(layer - Math.Max(indexInLayer, 0));
            if(layer % 2 == 0) 
            {
                var tmp = row;
                row = col;
                col = tmp;
            }
            break;
        case GridStyle.SPIRAL:
            if(index > 0)
            {
                int dx = 0;
                int dy = -1;
                for(int i=0; i<index; i++)
                {
                    if(row==col || (row < 0 && row == -col) || (row > 0 && row == 1-col))
                    {
                        int temp = dx;
                        dx = -dy;
                        dy = temp;
                    }
                    row += dx;
                    col += dy;
                }
            }
            break;
        case GridStyle.OFFSET:
            row = -(gridSize / 2) + index%gridSize;
            col = -(gridSize / 2) + MathHelper.FloorToInt(index/gridSize);
            break;
        default:
            row = index&gridSize;
            col = MathHelper.FloorToInt(index/gridSize);
            break;
    }
    return getMinePosition() + Vector3D.Multiply(forward, stepDist * row) + Vector3D.Multiply(getDirection(rotation + 1), stepDist * col);
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

// A passable model of the drone running this program, only independent feature is gravity levelling
public class Drone
{
    private IMyCubeGrid localGrid;
    private IMyProgrammableBlock brain;
    private IMyRemoteControl remote;
    private IMyShipConnector connector;
    private IMyShipConnector ejector;
    private List<IMyCargoContainer> cargo = new List<IMyCargoContainer>();
    private List<IMyTerminalBlock> inventories = new List<IMyTerminalBlock>();
    private List<IMyGyro> gyros = new List<IMyGyro>();
    private List<IMyShipDrill> drills = new List<IMyShipDrill>();
    private List<IMyBatteryBlock> batteries = new List<IMyBatteryBlock>();
    private IMySensorBlock sensor;
    
    private List<IMyThrust> thrusters = new List<IMyThrust>();
    private Dictionary<Base6Directions.Direction, List<IMyThrust>> thrustMap = new Dictionary<Base6Directions.Direction, List<IMyThrust>>();
    private PID pidX = new PID(), pidY = new PID(), pidZ = new PID();
    private Vector3D targetVelocity = new Vector3D(0, 0, 0);
    private PID pidYaw = new PID();
    
    Vector3D forwRef;
    Vector3D downRef;
    Vector3D gravTarget;
    MyGridProgram programTemp;
    
    public Drone(IMyProgrammableBlock me, MyGridProgram program)
    {
        localGrid = me.CubeGrid;
        brain = me;
        programTemp = program;
        
        List<IMyRemoteControl> controls = new List<IMyRemoteControl>();
        program.GridTerminalSystem.GetBlocksOfType<IMyRemoteControl>(controls);
        foreach(var control in controls)
            if(control.CubeGrid == localGrid)
            {
                control.CustomName = "TMT Drone Remote";
                remote = control;
                break;
            }
        
        List<IMyShipConnector> connectors = new List<IMyShipConnector>();
        program.GridTerminalSystem.GetBlocksOfType<IMyShipConnector>(connectors);
        foreach(var control in connectors)
            if(control.CubeGrid == localGrid)
                if(control.BlockDefinition.SubtypeId.ToLower().Contains("small"))
                {
                    control.CustomName = "TMT Ejector";
                    ejector = control;
                }
                else
                {
                    control.CustomName = "TMT Connector";
                    connector = control;
                }
        
        List<IMyCargoContainer> boxes = new List<IMyCargoContainer>();
        program.GridTerminalSystem.GetBlocksOfType<IMyCargoContainer>(boxes);
        foreach(var box in boxes)
            if(box.CubeGrid == localGrid)
            {
                box.CustomName = "TMT Cargo "+cargo.Count();
                cargo.Add(box);
            }
        List<IMyTerminalBlock> terminals = new List<IMyTerminalBlock>();
        program.GridTerminalSystem.GetBlocksOfType<IMyTerminalBlock>(terminals);
        foreach(var box in terminals)
            if(box.CubeGrid == localGrid && box.HasInventory)
                if(cargo.Count == 0 || cargo[0].GetInventory().CanTransferItemTo(box.GetInventory(), ores[0]))
                {
                    box.ShowInTerminal = false;
                    box.ShowInToolbarConfig = false;
                    inventories.Add(box);
                }
        
        List<IMyGyro> gyroscopes = new List<IMyGyro>();
        program.GridTerminalSystem.GetBlocksOfType<IMyGyro>(gyroscopes);
        foreach(var gyro in gyroscopes)
            if(gyro.CubeGrid == localGrid)
            {
                gyro.CustomName = "TMT Gyroscope "+gyros.Count();
                gyro.ShowInTerminal = false;
                gyro.ShowInToolbarConfig = false;
                gyros.Add(gyro);
            }
        
        List<IMyShipDrill> drillers = new List<IMyShipDrill>();
        program.GridTerminalSystem.GetBlocksOfType<IMyShipDrill>(drillers);
        foreach(var box in drillers)
            if(box.CubeGrid == localGrid)
            {
                box.CustomName = "TMT Drill "+drills.Count();
                box.ShowInTerminal = false;
                box.ShowInToolbarConfig = false;
                drills.Add(box);
            }
        
        List<IMyBatteryBlock> bats = new List<IMyBatteryBlock>();
        program.GridTerminalSystem.GetBlocksOfType<IMyBatteryBlock>(bats);
        foreach(var box in bats)
            if(box.CubeGrid == localGrid)
            {
                box.CustomName = "TMT Battery "+batteries.Count();
                box.ShowInTerminal = false;
                box.ShowInToolbarConfig = false;
                batteries.Add(box);
            }
        
        foreach(var direction in Base6Directions.EnumDirections)
            thrustMap[direction] = new List<IMyThrust>();
        
        if(remote != null)
        {
            List<IMyThrust> thrusters2 = new List<IMyThrust>();
            program.GridTerminalSystem.GetBlocksOfType<IMyThrust>(thrusters2);
            foreach(var thruster in thrusters2)
                if(thruster.CubeGrid == localGrid)
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
                        thruster.Enabled = true;
                        thruster.ThrustOverride = 0F;
                        thruster.CustomName = "TMT Aux Thruster";
                        continue;
                    }
                    
                    thrusters.Add(thruster);
                    registerThruster(thruster, localFace);
                    thruster.CustomName = "TMT Thruster "+(Enum.GetName(typeof(Base6Directions.Direction), localFace)[0]+""+getThrusters(localFace).Count);
                }
        }
        
        List<IMySensorBlock> sensors = new List<IMySensorBlock>();
        program.GridTerminalSystem.GetBlocksOfType<IMySensorBlock>(sensors);
        foreach(var sense in sensors)
            if(sense.CubeGrid == localGrid)
            {
                sense.CustomName = "TMT Collision Sensor";
                sense.Enabled = true;
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
    }
    
    public void tick()
    {
        if(remote != null && remote.IsFunctional && remote.IsUnderControl)
        {
            resetGyros();
            resetThrusters(thrusters);
            return;
        }
        updateOrientationRef();
        updateLevelling();
        
        if(remote != null && remote.IsFunctional && !(isUnderControl() || isConnected()))
        {
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
        
        setDump((getTotalMass() / getMaxMass()) >= 0.95F);
    }
    
    public Vector3D toLocal(Vector3D vectorIn){ return Vector3D.TransformNormal(vectorIn, MatrixD.Transpose(remote.WorldMatrix)); }
    public Vector3D toWorld(Vector3D vectorIn){ return Vector3D.TransformNormal(vectorIn, remote.WorldMatrix); }
    
    public bool runDiagnostic(Action<string> echo)
    {
        bool result = true;
        if(remote == null || !remote.IsFunctional)
        {
            echo(" X No operational remote control");
            result = false;
        }
        else if(!hasHomePosition())
        {
            echo(" X No home position set");
            result = false;
        }
        else if(!hasMinePosition())
        {
            echo(" X No mine position set");
            result = false;
        }
        
        if(gyros.Count == 0)
        {
            echo(" X No gyroscopes");
            result = false;
        }
        else
        {
            bool functional = false;
            foreach(var gyro in gyros)
                if(gyro != null && gyro.IsFunctional && gyro.Enabled)
                {
                    functional = true;
                    break;
                }
            
            if(!functional)
            {
                echo(" X No gyroscopes functional & enabled");
                result = false;
            }
        }
        
        if(connector == null || !connector.IsFunctional)
        {
            echo(" X No functional connector");
            result = false;
        }
        
        if(drills.Count == 0)
        {
            echo(" X No drills");
            result = false;
        }
        else
        {
            bool functional = false;
            foreach(var drill in drills)
                if(drill != null && drill.IsFunctional)
                {
                    functional = true;
                    break;
                }
            
            if(!functional)
            {
                echo(" X No functional drills");
                result = false;
            }
        }
        
        if(inventories.Count == 0)
        {
            echo(" X No containers");
            result = false;
        }
        else
        {
            bool functional = false;
            foreach(var box in inventories)
                if(box != null && box.IsFunctional)
                {
                    functional = true;
                    break;
                }
            
            if(!functional)
            {
                echo(" X  No functional containers");
                result = false;
            }
        }
        
        if(batteries.Count == 0)
        {
            echo(" X No batteries");
            result = false;
        }
        else
        {
            bool functional = false;
            foreach(var bat in batteries)
                if(bat != null && bat.IsFunctional && bat.Enabled && bat.ChargeMode == ChargeMode.Auto)
                {
                    functional = true;
                    break;
                }
            
            if(!functional)
            {
                echo(" X No functional & enabled batteries providing power");
                result = false;
            }
        }
        
        if(!hasCollisionSensor())
            echo(" ? No operational collision sensor");
        if(ejector == null || !ejector.IsFunctional || !ejector.Enabled)
            echo(" ? No operational ejector");
        
        return result;
    }
    
    public bool isUnderControl(){ return remote != null && remote.IsFunctional && remote.IsUnderControl; }
    
    public bool hasCollisionSensor(){ return sensor != null && sensor.Enabled && sensor.IsFunctional; }
    public bool collisionAlarm(){ return hasCollisionSensor() ? sensor.IsActive : false; }
    
    public List<IMyTerminalBlock> getCargo(){ return inventories; }
    public float getTotalMass(){ return remote == null ? 0F : remote.CalculateShipMass().TotalMass; }
    public float getMaxMass()
    {
        if(remote == null) return 0F;
        double gravity = getGravity().Length();
        
        float lift = 0F;
        List<IMyThrust> thrusters = getThrusters(Base6Directions.Direction.Up);
        foreach(var thruster in thrusters.GetRange(1, thrusters.Count-1))
            if(thruster != null && thruster.IsFunctional)
                lift += thruster.MaxThrust;
        
        return (float)(lift / gravity);
    }
    
    public bool isFull(){ return getTotalMass() >= getMaxMass(); }
    public bool isEmpty(){ return getCargoVolume() == 0F; }
    private float getCargoVolume()
    {
        float volume = 0F;
        foreach(var box in inventories)
        {
            if(box == null || !box.IsFunctional) continue;
            IMyInventory inv = box.GetInventory();
            volume += (float)inv.CurrentVolume.ToIntSafe();
        }
        return volume;
    }
    public bool hasAnyOf(MyItemType item)
    {
        foreach(var box in inventories)
            if(box.GetInventory().GetItemAmount(item).ToIntSafe() > 0)
                return true;
        return false;
    }
    public int getTotalOf(MyItemType item)
    {
        int tally = 0;
        foreach(var box in inventories)
            tally += box.GetInventory().GetItemAmount(item).ToIntSafe();
        return tally;
    }
    
    public void setDrills(bool on)
    {
        foreach(var drill in drills)
            if(drill != null && drill.IsFunctional)
                drill.Enabled = on;
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
    
    public float currentFacingError()
    {
        return getFacingError(moveForw);
    }
    
    public float getFacingError(Vector3D vector)
    {
        Vector3D forwLocal = toLocal(vector);
        double yawV = Math.Atan2(forwLocal.Z, forwLocal.X);
        double yawL = Math.Atan2(forwRef.Z, forwRef.X);
        return (float)MathHelper.ToDegrees(yawL - yawV) + (rotation * 90F);
    }
    
    private void updateLevelling()
    {
        resetGyros();
        double error = currentFacingError();
        double currentSpin = toLocal(remote.GetShipVelocities().AngularVelocity).Y;
        double spin = hasMinePos && Math.Abs(error) > 1 && currentSpin < 5 ? -MathHelper.Clamp(error, -10, 10) * 0.1D : 0D;
        
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
    
    public void resetGyros()
    {
        foreach(var gyro in gyros)
        {
            if(gyro == null || !gyro.Enabled || !gyro.IsFunctional) continue;
            gyro.GyroOverride = false;
            gyro.Pitch = 0F;
            gyro.Yaw = 0F;
            gyro.Roll = 0F;
        }
    }
    
    public IMyRemoteControl getRemote(){ return remote; }
    public Vector3D getGravity(){ return remote == null ? new Vector3D(0, 0, 0) : remote.GetNaturalGravity(); }
    
    public Vector3D getWorldPosition(){ return connector.GetPosition(); }
    
    public IMyShipConnector getConnector(){ return connector; }
    public void setConnector(bool on){ connector.Enabled = on; }
    public void setDump(bool on){ if(ejector != null && ejector.IsFunctional) ejector.CollectAll = ejector.ThrowOut = on; }
    public bool isConnected(){ return connector.Status == MyShipConnectorStatus.Connected; }
    public MyShipConnectorStatus getConnectorStatus(){ return connector.Status; }
    public void toggleConnect(){ connector.ToggleConnect(); }
    
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
    
    private void registerThruster(IMyThrust thruster, Base6Directions.Direction direction)
    {
        List<IMyThrust> set = thrustMap[direction];
        set.Add(thruster);
        thrustMap[direction] = set;
    }
    
    public void setTargetVelocity(Vector3D vel){ targetVelocity = vel; }
    public void clearTargetVelocity(){ targetVelocity = new Vector3D(0, 0, 0); }
    
    private List<IMyThrust> getThrusters(Base6Directions.Direction face)
    {
        return thrustMap[face];
    }
    
    private void resetThrusters(List<IMyThrust> group)
    {
        foreach(var thruster in group)
        {
            if(thruster == null) continue;
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
        foreach(var thruster in groupA) if(thruster != null && thruster.IsFunctional) tallyA++;
        
        int tallyB = 0;
        foreach(var thruster in groupB) if(thruster != null && thruster.IsFunctional) tallyB++;
        
        if(target != 0F)
        {
            double pidVal = pid.calculate(target, current) * mass;
            
            // Total thrust needed (N)
            double totalThrust = offset + pidVal;
            
            foreach(var thruster in groupA)
            {
                if(thruster == null || !thruster.IsFunctional) continue;
                thruster.Enabled = totalThrust > 0F;
                thruster.ThrustOverride = thruster.Enabled ? (float)totalThrust / tallyA : 0F;
            }
            
            totalThrust -= offset;
            foreach(var thruster in groupB)
            {
                if(thruster == null || !thruster.IsFunctional) continue;
                thruster.Enabled = totalThrust < 0F;
                thruster.ThrustOverride = thruster.Enabled ? (float)Math.Abs(totalThrust) / tallyB : 0F;
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

private abstract class State
{
    protected const double TOP_SPEED = 1D;
    
    private Controls[] affectedComponents = new Controls[0];
    
    protected State(Controls[] affectedIn){ affectedComponents = affectedIn; }
    
    // Performs any basic initial settings this state needs
    public virtual void enter(int index, int rotation, Drone model){ }
    // Performs the functions of this state
    public abstract void tick(int index, int rotation, Drone model, Action<string> echo);
    // Returns true if a state change should occur, ie. this state's work is finished and it can safely stop
    public abstract bool invalid(Drone model);
    // Resets any changes caused by this state and returns the state names that should be moved into
    public virtual String[] exit(Drone model)
    {
        resetModel(model);
        onExit(model);
        return exitStates(model);
    }
    public virtual void onExit(Drone model){ }
    public virtual String[] exitStates(Drone model){ return new String[]{"idle"}; }
    
    protected virtual void resetModel(Drone model)
    {
        foreach(var component in affectedComponents)
            switch(component)
            {
                case Controls.DRILLS: model.setDrills(false); break;
                case Controls.THRUSTERS: model.clearTargetVelocity(); break;
                case Controls.CONNECTOR: model.setConnector(true); break;
                case Controls.BATTERIES: model.resetBatteries(); break;
            }
    }
    
    protected enum Controls
    {
        DRILLS,
        THRUSTERS,
        CONNECTOR,
        BATTERIES
    }
}

private class StateDeposit : State
{
    public StateDeposit() : base(new Controls[]{Controls.BATTERIES}) { }
    
    public override void tick(int index, int rotation, Drone model, Action<string> echo)
    {
        model.doCharging();
        echo(" > Cargo Status:");
        foreach(var box in model.getCargo())
        {
            IMyInventory inv = box.GetInventory();
            double vol = (double)Math.Max(0,inv.CurrentVolume.ToIntSafe());
            if(vol <= 0D) continue;
            double max = (double)Math.Max(0, inv.MaxVolume.ToIntSafe());
            double fill = Math.Max(0, Math.Round(vol/max,2))*100;
            echo("   * "+box.CustomName+": "+(int)fill+"%");
        }
    }
    public override bool invalid(Drone model){ return model.isEmpty() && model.getTotalChargePercent() >= 0.8F; }
    public override String[] exitStates(Drone model){ return new String[]{"detach", "idle"}; }
}

// Detach from home connector and move away from it
private class StateDetach : State
{
    public StateDetach() : base(new Controls[]{Controls.CONNECTOR, Controls.THRUSTERS}) { }
    
    public override void enter(int index, int rotation, Drone model)
    {
        if(incFlag)
            increment();
    }
    
    public override void tick(int index, int rotation, Drone model, Action<string> echo)
    {
        if(index >= (gridSize * gridSize))
        {
            echo(" X Mining job completed");
            return;
        }
        if(model.getTotalMass() > model.getMaxMass())
        {
            echo(" X Cannot safely undock, insufficient thrust for lift");
            return;
        }
        
        if(model.isConnected())
            model.toggleConnect();
        model.setConnector(false);
        
        if(distToHome(model) < 2D)
        {
            Vector3D gravity = model.getGravity();
            gravity.Normalize();
            model.setTargetVelocity(Vector3D.Multiply(gravity, 1D));
        }
        else
            model.clearTargetVelocity();
    }
    
    public override bool invalid(Drone model)
    {
        return model.getConnectorStatus() == MyShipConnectorStatus.Unconnected && distToHome(model) >= 2D && model.getShipVelocities().Length() < 0.25D;
    }
    
    public override String[] exitStates(Drone model){ return new String[]{"travel_mine", "travel_home", "dock", "idle"}; }
    
    protected double distToHome(Drone model){ return (model.getWorldPosition() - getHomePosition()).Length(); }
}

// Move upwards towards home connector and dock with it
private class StateDock : StateDetach
{
    public StateDock(){ }
    
    public override void tick(int index, int rotation, Drone model, Action<string> echo)
    {
        echo(" > Connector: "+Enum.GetName(typeof(MyShipConnectorStatus), model.getConnectorStatus()));
        if(model.getConnectorStatus() == MyShipConnectorStatus.Connectable)
            model.toggleConnect();
        else if(distToHome(model) > 0.25D)
        {
            Vector3D gravity = model.getGravity();
            gravity.Normalize();
            model.setTargetVelocity(Vector3D.Multiply(gravity, -1D));
        }
        else
            model.clearTargetVelocity();
    }
    
    public override bool invalid(Drone model)
    {
        return model.isConnected();
    }
    
    public override String[] exitStates(Drone model)
    {
        if(!model.isConnected())
            return new String[]{"idle"};
        else
            return new String[]{"deposit", "detach", "travel_mine", "dig", "extract", "travel_home", "idle"};
    }
}

// Slowly descend whilst drills are on until cargo is full
private class StateDig : State
{
    bool isMining = false;
    Vector3D minePos;
    
    Vector3D digSite;
    
    public StateDig() : base(new Controls[]{Controls.THRUSTERS, Controls.DRILLS}) { }
    
    public bool canShipDig(Drone model){ return (model.getTotalMass() / model.getMaxMass()) < 0.9F && !model.isFull() && model.getTotalChargePercent() > 0.2F && (model.getWorldPosition()-digSite).Length() < boreDepth; }
    
    public override void tick(int index, int rotation, Drone model, Action<string> echo)
    {
        digSite = getMiningPosition();
        bool canDig = canShipDig(model);
        model.setDrills(canDig);
        bool empty = model.isEmpty();
        if((model.getWorldPosition() - digSite).Length() >= boreDepth)
            incFlag = true;
        
        if(canDig)
        {
            Vector3D gravity = model.getGravity();
            gravity.Normalize();
            
            double speed = 0.0125D;
            if(empty)
                if(model.hasCollisionSensor() && !model.collisionAlarm())
                    speed = 1D;
                else
                    speed = 0.5D;
            model.setTargetVelocity(Vector3D.Multiply(gravity, speed));
            echo(" > Depth "+Math.Round((model.getWorldPosition()-digSite).Length(), 2)+" ("+(isMining ? Math.Round((model.getWorldPosition() - minePos).Length(),2) : 0)+")");
            if(model.hasCollisionSensor())
                echo(" > Collision Alarm: "+model.collisionAlarm());
            echo(" > Lift Capacity: "+Math.Round(model.getTotalMass() / model.getMaxMass(),2)*100+"%");
            if(!isMining && !empty)
            {
                isMining = true;
                minePos = model.getWorldPosition();
            }
            else if(empty)
                isMining = false;
        }
        else
            model.clearTargetVelocity();
        
        if(!empty)
        {
            echo(" > Inventory");
            int stoneCount = model.getTotalOf(stone);
            int total = stoneCount;
            foreach(var item in ores)
                total += model.getTotalOf(item);
            
            foreach(var item in ores)
            {
                int count = model.getTotalOf(item);
                if(count > 0)
                    echo("   * "+item.SubtypeId+" "+count+" ("+Math.Round((float)count / (float)total, 2)*100+"%)");
            }
            if(stoneCount > 0)
                echo("   * "+stone.SubtypeId+" "+stoneCount+" ("+Math.Round((float)stoneCount / (float)total, 2)*100+"%)");
        }
    }
    
    public override bool invalid(Drone model){ return !canShipDig(model) && model.getShipVelocities().Length() < 0.1D; }
    
    public override void onExit(Drone model)
    {
        int oreCount = 0;
        foreach(var item in ores)
            oreCount += model.getTotalOf(item);
        if(oreCount <= 10000)
            incFlag = true;
    }
    
    public override String[] exitStates(Drone model){ return new String[]{"extract", "travel_home", "dock", "idle"}; }
}

// Base travelling state, moves the drone towards a destination laterally
private abstract class StateGo : State
{
    bool allowVertical;
    Vector3D destination;
    
    protected StateGo(bool verticalIn) : base(new Controls[]{Controls.THRUSTERS, Controls.CONNECTOR})
    {
        allowVertical = verticalIn;
    }
    
    public override void tick(int index, int rotation, Drone model, Action<string> echo)
    {
        destination = getDestination(index, rotation, model);
        Vector3D direction = (destination - model.getWorldPosition());
        double dist = distance(model);
        echo(" > Destination: "+vectorToString(destination));
        echo(" > Direction: "+vectorToString(direction));
        echo(" > Distance: "+Math.Round(dist,2));
        
        model.setConnector(false);
        
        float weight = (model.getTotalMass() / model.getMaxMass());
        bool needsEjector = weight > 0.95F;
        if(needsEjector)
            echo(" > Overweight: "+Math.Round(weight,2)*100+"%");
        if(dist > 0.5D && !needsEjector)
        {
            Vector3D motion = direction;
            motion.Normalize();
            
            double speed = model.hasCollisionSensor() && !model.collisionAlarm() ? TOP_SPEED * 2D : TOP_SPEED;
            speed = Math.Max(0.05D, Math.Min(speed, direction.Length()));
            model.setTargetVelocity(Vector3D.Multiply(motion, dist > 3D ? speed : 0.5D));
        }
        else
            model.clearTargetVelocity();
    }
    
    public override bool invalid(Drone model)
    {
        return distance(model) <= 0.5D && model.getShipVelocities().Length() < 0.25D;
    }
    
    protected abstract Vector3D getDestination(int index, int rotation, Drone model);
    
    private double distance(Drone model)
    {
        Vector3D direction = model.toLocal(destination - model.getWorldPosition());
        if(!allowVertical)
            direction.Y = 0D;
        return direction.Length();
    }
}

// Travel to calculated mining position, then dig
private class StateGoMine : StateGo
{
    public StateGoMine() : this(false){ }
    protected StateGoMine(bool verticalIn) : base(verticalIn){ }
    
    protected override Vector3D getDestination(int index, int rotation, Drone model)
    {
        Vector3D grav = model.getGravity();
        grav.Normalize();
        return getMiningPosition() + Vector3D.Multiply(grav, 2D);
    }
    
    public override String[] exitStates(Drone model){  return new String[]{"dig", "travel_home", "dock", "idle"}; }
}

// Travel from current position to home position
private class StateGoHome : StateGo
{
    public StateGoHome() : base(true){ }
    
    protected override Vector3D getDestination(int index, int rotation, Drone model)
    {
        Vector3D grav = model.getGravity();
        grav.Normalize();
        return getHomePosition() + Vector3D.Multiply(grav, 2D);
    }
    
    public override String[] exitStates(Drone model){ return new String[]{"dock", "idle"}; }
}

// Travel from current position to calculated mining position, with vertical thrust control permitted
private class StateExtract : StateGoMine
{
    public StateExtract() : base(true){ }
    
    public override String[] exitStates(Drone model){ return new String[]{"travel_home", "dock", "idle"}; }
}