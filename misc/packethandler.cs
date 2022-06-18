// === Inter-Grid Communication Test ===
const string signalChannel = "igc_test";
const PacketHandler.SystemType type = PacketHandler.SystemType.DEBUG;

PacketHandler handler;
List<long> listenerIds = new List<long>();

private int ticksRunning;
const String spinning = "-\\|/";

private String lastSent = "";

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
        handler.sendPacket(listenerIds, argument);
        lastSent = argument;
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
}

public void parseGlobal(string arg, long entityID){ }

public void parseLocal(string arg, long entityID)
{
    string[] data = arg.Split('|');
    if(data.Length >= 2 && data[0] == "response" && data[3] == Enum.GetName(typeof(PacketHandler.SystemType), type).ToLower())
        listenerIds.Add(entityID);
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
    
    public PacketHandler(String channelName, IMProgrammableBlock blockIn, MyGridProgram programIn, SystemType typeIn)
    {
        channel = channelName;
        parent = programIn;
		block = blockIn
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
		Vector3D blockPos = block.getPosition();
        return String.Join("|", 
			"response",
			block.CustomName,
			String.Join(",", blockPos.X, blockPos.Y, blockPos.Z)
			Enum.GetName(typeof(SystemType), type));
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