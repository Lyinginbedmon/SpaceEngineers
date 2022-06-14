// == Hermit Crab Delivery Drone OS ==
const string version = "1.6";

// Gyroscopes
List<IMyGyro> gyros = new List<IMyGyro>();
// Indicator light
IMyLightingBlock light;
// Terrain sensor
IMySensorBlock sensor;
// Docking connector
IMyShipConnector connector;
// Remote control
IMyRemoteControl remote;
// Thrusters mapped to their local grid directions
Dictionary<Base6Directions.Direction, List<IMyThrust>> thrustMap = new Dictionary<Base6Directions.Direction, List<IMyThrust>>();
// All thrusters
List<IMyThrust> thrusters = new List<IMyThrust>();
// All batteries
List<IMyBatteryBlock> batteries = new List<IMyBatteryBlock>();

int targetGPS = -1;
int maxSpeed = 50;
int minAltitude = 50;
bool ignoreAltitude = false;

string lightName = "Interior Light";

Vector3D forwRef;
Vector3D downRef;
Vector3D forwTarget = new Vector3D(0, 0, 0);
Vector3D gravTarget;
double distance = -1;

readonly Matrix directionMatrix = new Matrix(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
const double RadToDeg = 180 / Math.PI;

State state = State.STOP;
MoveStyle moveStyle = MoveStyle.LATERAL;
bool prevAltWarn = false;

bool isBeingControlled = false;

private enum MoveStyle
{
    FORWARD,
    LATERAL
}
private enum State
{
    STOP,    // Wait for further user input
    POINT,    // Rotate to face direction of destination
    MOVE,    // Move towards destination using forward thrusters
    RISE,    // Gain altitude
    LAND,    // Descend until minimum altitude or docking availability
    LAND2,    // LATERAL without altitude warning, for low-altitude landing corrections
    LATERAL,    // Use lateral thrusters at low velocity to minimise lateral distance to destination
    STABILISE    // Waiting for levelling code to steady the craft
}
readonly State[] STATES = new State[]{State.STOP, State.POINT, State.MOVE, State.RISE, State.LAND, State.LAND2, State.LATERAL};

public Program()
{
    Runtime.UpdateFrequency = UpdateFrequency.Update10;
    
    Me.CustomName = "Drone Control";
    
    List<IMyGyro> gyros2 = new List<IMyGyro>();
    GridTerminalSystem.GetBlocksOfType<IMyGyro>(gyros2);
    foreach(var gyro in gyros2)
        if(gyro.CubeGrid == Me.CubeGrid)
            gyros.Add(gyro);
    
    List<IMyRemoteControl> remotes = new List<IMyRemoteControl>();
    GridTerminalSystem.GetBlocksOfType<IMyRemoteControl>(remotes);
    foreach(var controller in remotes)
        if(controller.CubeGrid == Me.CubeGrid && controller.IsFunctional)
        {
            controller.CustomName = "Drone Remote";
            remote = controller;
            break;
        }
    
    foreach(var direction in Base6Directions.EnumDirections)
        thrustMap[direction] = new List<IMyThrust>();
    if(remote != null)
    {
        isBeingControlled = remote.IsUnderControl;
        
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
                else if(thrustVec == directionMatrix.Down)
                    localFace = Base6Directions.Direction.Down;
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
        
        List<IMyShipConnector> connectors = new List<IMyShipConnector>();
        GridTerminalSystem.GetBlocksOfType<IMyShipConnector>(connectors);
        foreach(var connect in connectors)
            if(connect.CubeGrid == Me.CubeGrid && getLocalFacing(connect) == directionMatrix.Up)
            {
                connect.CustomName = "Docking Connector";
                connect.Enabled = true;
                connect.ShowInToolbarConfig = false;
                
                connector = connect;
            }
    }
    
    List<IMySensorBlock> sensors = new List<IMySensorBlock>();
    GridTerminalSystem.GetBlocksOfType<IMySensorBlock>(sensors);
    foreach(var sense in sensors)
        if(sense.CubeGrid == Me.CubeGrid && sense.IsFunctional)
        {
            sense.CustomName = "Terrain Sensor";
            sense.Enabled = true;
            
            Vector3 faceVec = getLocalFacing(sense);
            sense.BackExtend = faceVec.Y <= 0 ? int.MaxValue : 0;
            sense.FrontExtend = faceVec.Y >= 0 ? int.MaxValue : 0;
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
    GridTerminalSystem.GetBlocksOfType<IMyBatteryBlock>(bats);
    foreach(var battery in bats)
        if(battery.CubeGrid == Me.CubeGrid)
        {
            battery.CustomName = "Battery "+batteries.Count;
            battery.ShowInTerminal = false;
            battery.ShowInToolbarConfig = false;
            batteries.Add(battery);
        }
    
    light = (IMyLightingBlock)GridTerminalSystem.GetBlockWithName(lightName);
    
    reset();
    
    parse(Storage);
}

public void Save()
{
    if(state != State.STOP)
        Storage = targetGPS + ";" + Enum.GetName(typeof(State), state);
    else
        Storage = "";
}

public void Main(string argument, UpdateType updateSource)
{
    Echo("Hermit Crab OS Version "+version);
    if(argument.Length > 0)
    {
        parse(argument);
        if(state == State.STOP)
            setState(State.RISE);
    }
    
    updateLevelling();
    updateOrientationRef();
    
    if(!diagnostic())
    {
        reset();
        return;
    }
    
    double prevDist = distance;
    
    Vector3D position = connector.GetPosition();
    double altitude = getAltitude();
    double velocity = remote.GetShipSpeed();
    
    if(targetGPS < 0 && state != State.STOP)
    {
        MyWaypointInfo pointA = getWaypoint(0);
        MyWaypointInfo pointB = getWaypoint(1);
        
        double distA = (pointA.Coords - position).Length();
        double distB = (pointB.Coords - position).Length();
        
        if(distA < distB)
            targetGPS = 0;
        else
            targetGPS = 1;
    }
    
    MyWaypointInfo waypoint = getWaypoint();
    
    Vector3D remoteWorld = remote.WorldMatrix.Translation;
    Vector3D target = waypoint.Coords;
    Vector3D destinationLocal = Vector3D.TransformNormal(target - remoteWorld, MatrixD.Transpose(remote.WorldMatrix));
    Vector3D connectorLocal = Vector3D.TransformNormal(connector.GetPosition() - remoteWorld, MatrixD.Transpose(remote.WorldMatrix));
    
    Vector3D travelVec = destinationLocal - connectorLocal;
    distance = travelVec.Length();
    double maxError = Math.Max(30D, distance / 100);
    
    Vector3D travelLateral = travelVec;
    travelLateral.Y = 0;
    double distanceLateral = travelLateral.Length();
    
    Vector3D forwWorld = Vector3D.TransformNormal(forwRef, remote.WorldMatrix);
    Vector3D travelForw = travelVec;
    travelForw.Normalize();
    double forwError = Math.Round((travelForw - forwWorld).Length(), 2);
    
    Vector3D rotation = remote.GetShipVelocities().AngularVelocity;
    rotation.Y = 0;
    double rotationSpeed = rotation.Length();
    
    sensor.Enabled = state != State.STOP;
    if(state != State.STOP)
    {
        Echo("Destination: "+waypoint.Name);
        Echo("Coords: "+waypoint.Coords);
        Echo("Distance: "+(int)distance);
        
        sensor.Enabled = true;
        if(rotationSpeed > 0.5D)
            setState(State.STABILISE);
    }
    else
    {
        int nextGPS = (int)clamp(targetGPS, 0, 1) == 0 ? 1 : 0;
        MyWaypointInfo nextDest = getWaypoint(nextGPS);
        
        Vector3D nextDestLocal = Vector3D.TransformNormal(target, MatrixD.Transpose(remote.WorldMatrix));
        Vector3D nextTravelVec = nextDestLocal - connectorLocal;
        
        Echo("Location: "+waypoint.Name);
        Echo("");
        Echo("Next stop: "+nextDest.Name);
        Echo("Coords: "+nextDest.Coords);
        Echo("Distance: "+(int)nextTravelVec.Length());
    }
    Echo("");
    Echo("Current state: ");
    if(isBeingControlled)
    {
        Echo(" > Being remotely controlled");
        
        if(isBeingControlled != remote.IsUnderControl)
        {
            isBeingControlled = remote.IsUnderControl;
            resetThrusters();
        }
    }
    else
        switch(state)
        {
            case State.STOP:
                Echo(" > Finished");
                Echo("   > "+(getTotalChargePercent() >= 0.8F ? "Launch ready" : "Recharging"));
                Echo("   > Charge: "+Math.Round(getTotalChargePercent(), 2)*100+"%");
                reset();
                
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
                
                if(light != null && !light.Enabled && getTotalChargePercent() >= 0.8F)
                {
                    targetGPS = (int)clamp(targetGPS, 0, 1) == 0 ? 1 : 0;
                    if(connector.Status == MyShipConnectorStatus.Connected)
                        connector.ToggleConnect();
                    setState(State.RISE);
                    setCharge(false);
                }
                else if(altitude > 5 && connector.Status != MyShipConnectorStatus.Connected)
                    setState(State.LAND);
                break;
            case State.STABILISE:
                Echo(" > Stabilising");
                
                if(rotationSpeed < 0.5D)
                    setState(State.POINT);
                break;
            case State.RISE:
                Echo(" > Increasing altitude");
                Echo("   > Altitude: "+(int)altitude);
                
                double speed = clamp(minAltitude - (int)altitude, 1, 100);
                if(collisionAlarm())
                    speed = 15;
                handleThrusters(speed, remote.GetShipVelocities().LinearVelocity.Y, getThrusters(Base6Directions.Direction.Up), getThrusters(Base6Directions.Direction.Down));
                
                if(!lowAltitudeWarning(0))
                    setState(State.POINT);
                break;
            case State.POINT:
                if(moveStyle == MoveStyle.LATERAL)
                {
                    setState(State.MOVE);
                    return;
                }
                Echo(" > Rotating towards destination");
                Echo("   > Target:");
                Echo("     > X "+Math.Round(forwTarget.X, 2));
                Echo("     > Z "+Math.Round(forwTarget.Z, 2));
                
                Vector3D currentForw = forwRef;
                currentForw.Y = 0;
                Echo("   > Current:");
                Echo("     > X "+Math.Round(currentForw.X, 2));
                Echo("     > Z "+Math.Round(currentForw.Z, 2));
                Echo("   > Deviation: "+forwError);
                
                // FIXME Set forwTarget to target direction and rotate accordingly
                
                if(lowAltitudeWarning(10))
                    setState(State.RISE);
                else if(forwError <= 0.02D)
                    setState(State.MOVE);
                else if(distanceLateral < 20)
                    setState(State.LATERAL);
                
                if(state != State.POINT)
                    forwTarget = new Vector3D(0, 0, 0);
                break;
            case State.MOVE:
                Echo(" > Moving towards destination");
                Echo("   > Move style: "+Enum.GetName(typeof(MoveStyle), moveStyle));
                Echo("   > Distance: "+(int)distanceLateral);
                setIndicator(true, 2);
                
                int speedLimit = (int)Math.Min(maxSpeed, distanceLateral / 10);
                Echo("   > Max Speed: "+speedLimit);
                Echo("   > Velocity: "+(int)velocity);
                switch(moveStyle)
                {
                    case MoveStyle.FORWARD:
                        handleThrusters(speedLimit, remote.GetShipSpeed(), getThrusters(Base6Directions.Direction.Forward), getThrusters(Base6Directions.Direction.Backward));
                        
                        if(prevDist >= 0 && distance > prevDist || forwError > 0.1D)
                            setState(State.POINT);
                        break;
                    case MoveStyle.LATERAL:
                        moveLateral(travelLateral, remote.GetShipVelocities().LinearVelocity, Math.Min(maxSpeed, distanceLateral / 10), 1D);
                        break;
                }
                
                if(lowAltitudeWarning(10))
                    setState(State.RISE);
                else if(distanceLateral < 20)
                    setState(State.LATERAL);
                break;
            case State.LATERAL:
                Echo(" > Final approach");
                Echo("   > Velocity: "+(int)velocity);
                Echo("   > Distance: "+(int)distanceLateral);
                
                double maxVel = Math.Floor(Math.Min(travelLateral.Length(), altitude) / 10);
                moveLateral(travelLateral, remote.GetShipVelocities().LinearVelocity, maxVel);
                
                if(lowAltitudeWarning(10))
                    setState(State.RISE);
                else if(distanceLateral > 25)
                    setState(State.MOVE);
                else if(distanceLateral < 0.5D)
                    setState(State.LAND);
                break;
            case State.LAND:
                Echo(" > Landing");
                
                Vector3D travelVertical = travelVec;
                travelVertical.X = 0;
                travelVertical.Z = 0;
                double distanceVertical = travelVertical.Length();
                Echo("   > Altitude: "+(int)distanceVertical);
                Echo("   > Deviation: "+(int)distanceLateral);
                setIndicator(true, 1);
                
                moveLateral(travelLateral, remote.GetShipVelocities().LinearVelocity, 1D);
                if(distanceVertical > 1)
                    handleThrusters(collisionAlarm() ? Math.Min(2D, distanceVertical) : 15D, remote.GetShipVelocities().LinearVelocity.Y, getThrusters(Base6Directions.Direction.Down), getThrusters(Base6Directions.Direction.Up));
                else
                {
                    resetThrusters(getThrusters(Base6Directions.Direction.Down));
                    resetThrusters(getThrusters(Base6Directions.Direction.Up));
                }
                    
                
                if((!ignoreAltitude && altitude < 5) || connector.Status != MyShipConnectorStatus.Unconnected)
                {
                    if(connector.Status == MyShipConnectorStatus.Connectable)
                        connector.ToggleConnect();
                    setIndicator(true, 0);
                    setState(State.STOP);
                }
                else if(distanceLateral > 1 && prevAltWarn != collisionAlarm())
                    setState(State.LAND2);
                break;
            case State.LAND2:
                Echo(" > Landing correction");
                Echo("   > Velocity: "+(int)velocity);
                Echo("   > Distance: "+(int)distanceLateral);
                
                double maxVel2 = Math.Floor(Math.Min(travelLateral.Length(), altitude) / 10);
                moveLateral(travelLateral, remote.GetShipVelocities().LinearVelocity, maxVel2);
                
                if(distanceLateral < 0.5D)
                    setState(State.LAND);
                break;
        }
    prevAltWarn = collisionAlarm();
    handleIndicator(state);
}

private void setState(State stateIn)
{
    state = stateIn;
    reset();
}

// #### UTILITY FUNCTIONS ####

private void parse(string varString)
{
    if(varString.Length == 0)
        return;
    
    Echo("Parsing "+varString);
    targetGPS = -1;
    
    var parts = varString.Split(';');
    for(int i=0; i < parts.Length; i++)
    {
        string val = parts[i];
        if(val.Length == 0)
            continue;
        switch(i)
        {
            case 0:
                targetGPS = (int)clamp(int.Parse(val), -1, 1);
                Echo("Read GPS "+targetGPS);
                break;
            case 1:
                state = State.STOP;
                foreach(var status in STATES)
                    if(Enum.GetName(typeof(State), status).ToLower() == val.ToLower())
                    {
                        state = status;
                        break;
                    }
                Echo("Read state "+state);
                break;
            case 2:
                maxSpeed = int.Parse(val);
                Echo("Read max speed "+maxSpeed);
                break;
            case 3:
                minAltitude = int.Parse(val);
                Echo("Read minimum altitude "+minAltitude);
                break;
        }
    }
}

// Resets all gyroscopes and thrusters
private void reset()
{
    distance = -1;
    if(remote != null)
        remote.DampenersOverride = true;
    resetGyros();
    thrustOverride(0F, thrusters);
    enableThrusters(true, thrusters);
}

// Runs a diagnostic of all vital components and returns true if they all pass
private bool diagnostic()
{
    bool pass = true;
    if(remote == null)
    {
        Echo(" X Missing remote control");
        pass = false;
    }
    else
    {
        List<MyWaypointInfo> waypoints = new List<MyWaypointInfo>();
        remote.GetWaypointInfo(waypoints);
        
        if(waypoints.Count < 2)
        {
            Echo(" X Not enough waypoints registered");
            pass = false;
        }
    }
    
    foreach(var direction in Base6Directions.EnumDirections)
    {
        List<IMyThrust> set = getThrusters(direction);
        string group = Enum.GetName(typeof(Base6Directions.Direction), direction);
        if(set.Count == 0)
        {
            Echo(" X Missing thrusters for "+group);
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
                Echo(" X No functional thrusters for "+group);
                pass = false;
            }
        }
    }
    
    if(gyros.Count == 0)
    {
        Echo(" X Missing gyroscopes");
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
            Echo(" X No functional gyroscopes");
            pass = false;
        }
    }
    
    if(sensor == null || !sensor.IsFunctional)
    {
        Echo(" X No terrain collision sensor");
        pass = false;
    }
    
    if(connector == null || !connector.IsFunctional)
    {
        Echo(" X No functional docking connector");
        pass = false;
    }
    
    if(light == null)
        Echo(" ? Missing indicator light named "+lightName);
    
    return pass;
}

public void setCharge(bool recharge)
{
    foreach(var battery in batteries)
        battery.ChargeMode = recharge ? ChargeMode.Recharge : ChargeMode.Auto;
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

public float getAvailableChargePercent()
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

// Sets the indicator light
private void setIndicator(bool on, int blink)
{
    if(light == null)
        return;
    light.Enabled = on;
    light.BlinkIntervalSeconds = blink;
}

private void handleIndicator(State state)
{
    light.Enabled = true;
    Color lightColour = new Color(255,255,255);
    switch(state)
    {
        case State.RISE:    lightColour = new Color(255,0,0); break;
        case State.POINT:    lightColour = new Color(255,165,0); break;
        case State.MOVE:    lightColour = new Color(255,255,0); break;
        case State.LATERAL:    lightColour = new Color(0,255,0); break;
        case State.LAND:    lightColour = new Color(0,255,255); break;
        case State.LAND2:    lightColour = new Color(0,0,255); break;
        case State.STABILISE:    lightColour = new Color(255,0,255); break;
        default:    lightColour = new Color(255,255,255); break;
    }
    light.Color = lightColour;
}

private float clamp(float val, float min, float max)
{
    float minVal = Math.Min(min, max);
    float maxVal = Math.Max(min, max);
    return Math.Max(minVal, Math.Min(maxVal, val));
}

private MyWaypointInfo getWaypoint()
{
    return getWaypoint(targetGPS);
}

private MyWaypointInfo getWaypoint(int index)
{
    List<MyWaypointInfo> waypoints = new List<MyWaypointInfo>();
    remote.GetWaypointInfo(waypoints);
    index = (int)clamp(index, 0, waypoints.Count - 1);
    
    return waypoints[index];
}

// #### ROTATION HANDLING ####

// Sets gyroscopes to maintain gravity level on Pitch and Roll axises
private void updateLevelling()
{
    foreach(var gyro in gyros)
    {
        Matrix localOrientation;
        gyro.Orientation.GetMatrix(out localOrientation);
        var axisPR = getGyroVec(gravTarget, downRef, gyro.WorldMatrix.GetOrientation(), localOrientation);
        var axisY = getGyroVec(forwTarget, forwRef, gyro.WorldMatrix.GetOrientation(), localOrientation);
        
        gyro.GyroOverride = true;
        gyro.Pitch = (float)-axisPR.X;
        gyro.Yaw = (float)-axisY.Y;
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
    
    forwRef = Vector3D.Transform(shipOrientation.Forward, quatPitch * quatRoll);
    forwRef.Y = 0;
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

private bool collisionAlarm(){ return sensor.Enabled && sensor.IsActive; }

private bool lowAltitudeWarning(int offset)
{
    return collisionAlarm() || !ignoreAltitude && getAltitude() < (minAltitude - offset);
}

private void handleThrusters(double topSpeed, double speed, List<IMyThrust> positive, List<IMyThrust> negative)
{
    enableThrusters(false, negative);
    
    double minSpeed = Math.Max(0.02D, topSpeed - 5);
    
    if(speed <= minSpeed)
        thrustOverride(1, positive);
    else if(speed > topSpeed)
    {
        thrustOverride(0, positive);
        enableThrusters(true, negative);
    }
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

// Sets the thrust override in all forward thrusters to a percentage of maximum
private void thrustOverride(float vol, List<IMyThrust> blocks)
{
    foreach(var thruster in blocks)
        thruster.ThrustOverride = thruster.MaxThrust * vol;
}

private void resetThrusters()
{
    resetThrusters(thrusters);
}

private void resetThrusters(List<IMyThrust> blocks)
{
    enableThrusters(true, blocks);
    thrustOverride(0F, blocks);
}

private void moveLateral(Vector3D moveVec, Vector3D velocities, double topSpeed, double minSpeed)
{
    Vector3D normal = moveVec;
    normal.Normalize();
    
    double maxX = Math.Max(minSpeed, Math.Min(topSpeed, Math.Abs(moveVec.X)));
    double maxY = Math.Max(minSpeed, Math.Min(topSpeed, Math.Abs(moveVec.Y)));
    double maxZ = Math.Max(minSpeed, Math.Min(topSpeed, Math.Abs(moveVec.Z)));
    
    if(moveVec.X != 0)
        Echo("     > X "+Math.Abs(Math.Round(moveVec.X, 2))+" ("+Math.Round(Math.Abs(velocities.X),2)+" / "+Math.Round(maxX,2)+")");
    if(moveVec.Y != 0)
        Echo("     > Y "+Math.Abs(Math.Round(moveVec.Y, 2))+" ("+Math.Round(Math.Abs(velocities.Y),2)+" / "+Math.Round(maxY,2)+")");
    if(moveVec.Z != 0)
        Echo("     > Z "+Math.Abs(Math.Round(moveVec.Z, 2))+" ("+Math.Round(Math.Abs(velocities.Z), 2)+" / "+Math.Round(maxZ,2)+")");
    
    foreach(var thruster in thrusters)
    {
        Vector3 thrustVector = getLocalFacing(thruster);
        if(normal.X != 0 && Math.Sign(thrustVector.X) == Math.Sign(normal.X) && Math.Abs(velocities.X) < maxX)
            thruster.ThrustOverride = thruster.MaxThrust * (float)Math.Abs(normal.X);
        else if(normal.Y != 0 && Math.Sign(thrustVector.Y) == Math.Sign(normal.Y) && Math.Abs(velocities.Y) < maxY)
            thruster.ThrustOverride = thruster.MaxThrust * (float)Math.Abs(normal.Y);
        else if(normal.Z != 0 && Math.Sign(thrustVector.Z) == Math.Sign(normal.Z) && Math.Abs(velocities.Z) < maxZ)
            thruster.ThrustOverride = thruster.MaxThrust * (float)Math.Abs(normal.Z);
        else
            thruster.ThrustOverride = 0F;
    }
}

private void moveLateral(Vector3D moveVec, Vector3D velocities, double topSpeed)
{
    moveLateral(moveVec, velocities, topSpeed, 0.05F);
}