// === Inter-Grid Communication Test ===
const string signalChannel = "igc_test";
const PacketHandler.SystemType type = PacketHandler.SystemType.DEBUG;

PacketHandler handler;
List<long> listenerIds = new List<long>();

private int ticksRunning;
const String spinning = "-\\|/";

public Program()
{
    Runtime.UpdateFrequency = UpdateFrequency.Update10;
    handler = new PacketHandler(signalChannel, Me, this, type);
    handler.ping();
}

public void Main(string argument, UpdateType updateSource)
{
    Echo("Running "+spinning[++ticksRunning % spinning.Length]);
    
    handler.tick(parseGlobal, parseLocal);
    Echo(listenerIds.Count+" receivers");
    if(argument.Length > 0)
    {
        if(argument.ToLower() == "ping")
        {
            listenerIds.Clear();
            handler.ping();
        }
        else
            handler.sendPacket(listenerIds, argument);
    }
    
    if(handler.lastPacketSent().Length > 0)
    {
        Echo("Packet sent:");
        foreach(var line in handler.lastPacketSent().ToLower().Split('|'))
        Echo(" * "+line);
    }
    
    if(handler.lastPacketReceived().Length > 0)
    {
        Echo("Packet received:");
        string[] data = handler.lastPacketReceived().ToLower().Split('|');
        Echo(" * "+data[0]);
        Echo(" * "+data[1]);
        string[] vecNum = data[2].Split(',');
        Vector3D pos = new Vector3D(double.Parse(vecNum[0]), double.Parse(vecNum[1]), double.Parse(vecNum[2]));
        Echo(" * "+String.Join(", ", Math.Round(pos.X,2), Math.Round(pos.Y,2), Math.Round(pos.Z,2)));
        Echo(" * "+data[3]);
    }
    
    // Clean the listener list of objects we aren't connected to anymore
    List<long> detached = new List<long>();
    detached.AddRange(listenerIds);
    foreach(var id in detached)
        if(!isConnectedTo(id))
            listenerIds.Remove(id);
}

public void parseGlobal(string arg, long entityID){ }

public void parseLocal(string arg, long entityID)
{
    string[] data = arg.Split('|');
    if(data[0] == "response" && data[3] == Enum.GetName(typeof(PacketHandler.SystemType), type).ToLower() && isPartOfShip(entityID, false))
        listenerIds.Add(entityID);
}

// Returns true if the given entity ID exists on this or any connected grid
public bool isConnectedTo(long entityID)
{
    return GridTerminalSystem.GetBlockWithId(entityID) != null;
}

/**
 * Returns true only if the given entity ID exists on a grid connected via non-static grids
 * Used to identify if a given block exists as part of a connected ship, rather than a station it might be docked to
 * VERY loop-heavy, try to only use this during initial identification and then use isConnectedTo instead
 */
public bool isPartOfShip(long entityID, bool ignoreStatics)
{
    if(entityID <= 0) return false;
    IMyTerminalBlock target = GridTerminalSystem.GetBlockWithId(entityID);
    if(target == null) return false;
    
    IMyCubeGrid targetGrid = target.CubeGrid;
    if(targetGrid == Me.CubeGrid) return true;
    
    // Create dictionary of all connected grids to their connected connectors
    List<IMyShipConnector> connectors = new List<IMyShipConnector>();
    GridTerminalSystem.GetBlocksOfType<IMyShipConnector>(connectors);
    Dictionary<IMyCubeGrid, List<IMyShipConnector>> connectorMap = new Dictionary<IMyCubeGrid, List<IMyShipConnector>>();
    foreach(var connector in connectors)
    {
        IMyCubeGrid grid = connector.CubeGrid;
        if((grid != Me.CubeGrid && grid.IsStatic && ignoreStatics) || connector.Status != MyShipConnectorStatus.Connected)
            continue;
        List<IMyShipConnector> set = connectorMap.ContainsKey(grid) ? connectorMap[grid] : new List<IMyShipConnector>();
        set.Add(connector);
        connectorMap[grid] = set;
    }
    
    // Step through the connector dictionary until we encounter the target grid, if at all
    List<IMyCubeGrid> visitedGrids = new List<IMyCubeGrid>();
    List<IMyCubeGrid> search = new List<IMyCubeGrid>();
    search.Add(Me.CubeGrid);
    
    while(search.Count > 0)
    {
        List<IMyCubeGrid> nextSearch = new List<IMyCubeGrid>();
        foreach(var grid in search)
        {
            if(grid == targetGrid || grid.IsSameConstructAs(targetGrid))
                return true;
            visitedGrids.Add(grid);
            
            if(connectorMap.ContainsKey(grid))
                foreach(var connector in connectorMap[grid])
                {
                    IMyShipConnector linkedTo = connector.OtherConnector;
                    if(linkedTo == null) continue;
                    
                    IMyCubeGrid linking = linkedTo.CubeGrid;
                    // We never encountered this grid in our dictionary, probably because it's static
                    if(!connectorMap.ContainsKey(linking))
                        continue;
                    // We've already encountered this grid or are going to visit it later in this search
                    if(visitedGrids.Contains(linking) || nextSearch.Contains(linking) || search.Contains(linking))
                        continue;
                    nextSearch.Add(linking);
                }
        }
        search.Clear();
        search.AddRange(nextSearch);
    }
    return false;
}

public class PacketHandler
{
    private string channel;
    private MyGridProgram parent;
    private IMyProgrammableBlock block;
    private SystemType type;
    
    private IMyUnicastListener earLocal;
    private IMyBroadcastListener earGlobal;
    
    private string lastSent = "";
    private string lastReceived = "";
    
    public PacketHandler(String channelName, IMyProgrammableBlock blockIn, MyGridProgram programIn, SystemType typeIn)
    {
        channel = channelName;
        parent = programIn;
        block = blockIn;
        type = typeIn;
        
        earGlobal = parent.IGC.RegisterBroadcastListener(channel);
        earGlobal.SetMessageCallback(channel);
        earLocal = parent.IGC.UnicastListener;
    }
    
    public string lastPacketSent(){ return lastSent; }
    public string lastPacketReceived(){ return lastReceived; }
    
    public void tick(Action<string,long> parseGlobal, Action<string,long> parseLocal)
    {
        while(earGlobal.HasPendingMessage)
        {
            var message = earGlobal.AcceptMessage();
            if(message.Tag == channel)
            {
                lastReceived = message.Data.ToString().ToLower();
                if(lastReceived == "ping")
                    sendPacket(message.Source, getPingResponse());
                else
                    parseGlobal(lastReceived, message.Source);
            }
        }
        
        while(earLocal.HasPendingMessage)
        {
            var message = earLocal.AcceptMessage();
            if(message.Tag == channel)
            {
                lastReceived = message.Data.ToString();
                parseLocal(lastReceived, message.Source);
            }   
        }
    }
    
    public String getPingResponse()
    {
        Vector3D blockPos = block.GetPosition();
        return String.Join("|", 
            "response",
            block.CustomName,
            String.Join(",", blockPos.X, blockPos.Y, blockPos.Z),
            Enum.GetName(typeof(SystemType), type).ToLower());
    }
    
    public void ping(){ sendPacket("ping"); }
    public void sendPacket(String data){ sendPacket(data, TransmissionDistance.AntennaRelay); }
    public void sendPacket(String data, TransmissionDistance range)
    {
        parent.IGC.SendBroadcastMessage(channel, data, range);
        lastSent = data;
    }
    public void sendPacket(List<long> ids, String data){ foreach(var id in ids) sendPacket(id, data); }
    public void sendPacket(long entityID, String data)
    {
        parent.IGC.SendUnicastMessage(entityID, channel, data);
        lastSent = data;
    }
    
    public enum SystemType
    {
        COMBAT,
        DEFENSE,
        SHIPPING,
        MANAGEMENT,
        UTILITY,
        MISC,
        DEBUG
    }
}