// == Creation Item Sorting OS ==

// HOW TO USE
// The default config is automatically loaded into the programmable block's CustomData if it is blank when compiled
// The program will automatically shut off in this event to give you a moment to set it up first
// Adjust the config however you like
// * Prefix - Used to indicate blocks being manipulated by this system
// * StorageFlag - The value in a block's CustomData that flags it as destination storage. Blocks can have multiple flags
// * InputFlag - The value in a block's CustomData that flags it as input storage. Connectors and Cargo Terminals are included automatically.
// Once the config has been adjusted, recompile the program. A spinning icon should appear in the console to indicate activity
//
// SETTING UP THE NETWORK
// Go to each storage inventory you want to be included, and put the StorageFlag value in its CustomData (main_hold by default)
// Go to each input inventory you want to be included, and put the InputFlag value in its CustomData (auto_assembler by default)
// Then recompile the programmable block to apply the updates
// 
// CONSOLE COMMANDS
// * reset_config - Reverts the block's CustomData to the default config setup, does NOT reload the config
// * reload_config - Loads the config from the block's CustomData without recompiling
// * scan_storage - Rescans the local grid for any storage inventories of note
// * scan_input - Rescans the local grid for any input inventories of note
// * stop - Immediately halts the program, recompile to restart it

const String version = "1.0";
const String spinning = "-\\|/";
String config_default = "";

String prefix;
String storageFlag;
String inputFlag;
// All cargo containers
List<IMyTerminalBlock> itemStorage = new List<IMyTerminalBlock>();

// All input inventories
List<IMyTerminalBlock> itemInputs = new List<IMyTerminalBlock>();
// All input producers
List<IMyProductionBlock> itemProducers = new List<IMyProductionBlock>();

int ticksRunning = 0;

private static MyIni config = new MyIni();

public Program()
{
    Runtime.UpdateFrequency = UpdateFrequency.Update100;
    createDefaultConfig();
    start();
}

public void start()
{
    // Define component thresholds
    loadConfig(Me.CustomData);
    
    // Identify input and storage inventories
    collectInventories();
}

private void createDefaultConfig()
{
    MyIni defConfig = new MyIni();
    
    defConfig.AddSection("general");
    defConfig.Set("general", "Prefix", "CRT");
    defConfig.Set("general", "StorageFlag", "main_hold");
    defConfig.Set("general", "InputFlag", "auto_assembler");
    
    config_default = defConfig.ToString();
}

public void loadConfig(String customData)
{
    // If no config is present, load the default
    if(customData.Length == 0)
    {
        Runtime.UpdateFrequency = UpdateFrequency.Once;
        Me.CustomData = config_default;
        throw new Exception("Blank config detected, resetting");
    }
    
    MyIniParseResult result;
    if(!config.TryParse(customData, out result))
    {
        Runtime.UpdateFrequency = UpdateFrequency.Once;
        throw new Exception(result.ToString());
    }
    
    // Identify system variables
    prefix = config.Get("general", "Prefix").ToString("CRT");
    storageFlag = config.Get("general", "StorageFlag").ToString("main_hold").ToLower();
    inputFlag = config.Get("general", "InputFlag").ToString("auto_assembler").ToLower();
    
    Me.CustomName = prefix+" Item Sorting System";
}

public void Main(string argument, UpdateType updateSource)
{
    ++ticksRunning;
    Echo("Creation Item Sorting "+version+" "+getSpinning());
    Echo(" * "+(itemInputs.Count + itemProducers.Count)+" item inputs");
    
    int tally = 0;
    foreach(var box in itemInputs)
        tally += getTotalItems(box.GetInventory());
    foreach(var box in itemProducers)
        tally += getTotalItems(box.OutputInventory);
    Echo("  - "+tally+" total items waiting for sorting");
    
    Echo(" * "+itemStorage.Count+" storage containers");
    
    if(tally > 0)
    {
        evaluateInputs(itemInputs, itemProducers);
        Runtime.UpdateFrequency = UpdateFrequency.Update10;
    }
    else
        Runtime.UpdateFrequency = UpdateFrequency.Update100;
    
    if(argument.Length > 0)
    {
        string command = argument.ToLower();
        if(command == "reset_config")
            Me.CustomData = config_default;
        else if(command == "reload_config")
            start();
        else if(command == "scan_storage")
            collectContainers();
        else if(command == "scan_input")
            collectInputs();
        else if(command == "stop")
        {
            Runtime.UpdateFrequency = UpdateFrequency.Once;
            throw new Exception("Script stopped by command");
        }
    }
}

// #### MANAGEMENT FUNCTIONS ####

public void evaluateInputs(List<IMyTerminalBlock> inputs, List<IMyProductionBlock> producers)
{
    foreach(var input in inputs)
        if(processInput(input.GetInventory()))
            return;
    
    foreach(var producer in producers)
        if(processInput(producer.OutputInventory))
            return;
}

public bool processInput(IMyInventory inv)
{
    // Identify items present in input
    int index = 0;
    while(index < inv.ItemCount)
    {
        MyInventoryItem contents = inv.GetItemAt(index++).Value;
        if(contents.Amount <= 0)
            continue;
        
        // Try to sort item into storage inventories
        if(tryStore(contents, inv))
            return true;
    }
    return false;
}

public bool tryStore(MyInventoryItem item, IMyInventory inv)
{
    List<IMyTerminalBlock> boxes = new List<IMyTerminalBlock>();
    ListExtensions.AddList(boxes, itemStorage);
    bool result = false;
    MyItemType type = item.Type;
    while(boxes.Count > 0 && item.Amount > 0)
    {
        // Find box with highest existing volume of item
        IMyTerminalBlock box = null;
        int boxTotal = -1;
        foreach(IMyTerminalBlock b in boxes)
            if(b.GetInventory().IsFull || !inv.IsConnectedTo(b.GetInventory()))
                continue;
            else if(box == null || b.GetInventory().GetItemAmount(type).ToIntSafe() > boxTotal)
            {
                box = b;
                boxTotal = box.GetInventory().GetItemAmount(type).ToIntSafe();
            }
        
        // Remove box from list and attempt to transfer to it
        boxes.Remove(box);
        result = inv.TransferItemTo(box.GetInventory(), item, item.Amount) || result;
    }
    return result;
}

public static int compare(IMyTerminalBlock a, IMyTerminalBlock b, MyItemType item)
{
    int x = a.GetInventory().GetItemAmount(item).ToIntSafe();
    int y = b.GetInventory().GetItemAmount(item).ToIntSafe();
    return x < y ? -1 : x > y ? 1 : 0;
}

// #### UTILITY FUNCTIONS ####

// Returns the spinning progress icon
public char getSpinning()
{
    return spinning[this.ticksRunning % spinning.Length];
}

public void collectInventories()
{
    itemInputs.Clear();
    itemStorage.Clear();
    List<IMyCargoContainer> cargo = new List<IMyCargoContainer>();
    GridTerminalSystem.GetBlocksOfType<IMyCargoContainer>(cargo);
    foreach(var box in cargo)
        if(box.CubeGrid == Me.CubeGrid)
        {
            string type = box.BlockDefinition.SubtypeName;
            if(isBlockFlagged(box, storageFlag))
                itemStorage.Add(box);
            else if(isBlockFlagged(box, inputFlag) || type.Contains("CargoTerminal"))
                itemInputs.Add(box);
        }
    
    // Add all grid-connected connectors to the monitoring list as writ
    List<IMyShipConnector> connectors = new List<IMyShipConnector>();
    GridTerminalSystem.GetBlocksOfType<IMyShipConnector>(connectors);
    foreach(var box in connectors)
        if(box.CubeGrid == Me.CubeGrid)
            itemInputs.Add(box);
    
    // Add all nominated production blocks (assemblers, refineries, etc.)
    itemProducers.Clear();
    List<IMyProductionBlock> producers = new List<IMyProductionBlock>();
    GridTerminalSystem.GetBlocksOfType<IMyProductionBlock>(producers);
    foreach(var box in producers)
        if(box.CubeGrid == Me.CubeGrid && isBlockFlagged(box, inputFlag))
            itemProducers.Add(box);
}

// Collects all nominated cargo containers, as well as all cargo terminals and connectors, on this grid
public void collectContainers()
{
    itemStorage.Clear();
    List<IMyCargoContainer> cargo = new List<IMyCargoContainer>();
    GridTerminalSystem.GetBlocksOfType<IMyCargoContainer>(cargo);
    foreach(var box in cargo)
        if(box.CubeGrid == Me.CubeGrid)
        {
            string type = box.BlockDefinition.SubtypeName;
            if(isBlockFlagged(box, storageFlag))
                itemStorage.Add(box);
        }
}

public void collectInputs()
{
    itemInputs.Clear();
    itemProducers.Clear();
    List<IMyCargoContainer> cargo = new List<IMyCargoContainer>();
    GridTerminalSystem.GetBlocksOfType<IMyCargoContainer>(cargo);
    foreach(var box in cargo)
        if(box.CubeGrid == Me.CubeGrid)
        {
            string type = box.BlockDefinition.SubtypeName;
            if(!isBlockFlagged(box, storageFlag) && isBlockFlagged(box, inputFlag) || type.Contains("CargoTerminal"))
                itemInputs.Add(box);
        }
    
    // Add all grid-connected connectors to the monitoring list as writ
    List<IMyShipConnector> connectors = new List<IMyShipConnector>();
    GridTerminalSystem.GetBlocksOfType<IMyShipConnector>(connectors);
    foreach(var box in connectors)
        if(box.CubeGrid == Me.CubeGrid)
            itemInputs.Add(box);
    
    // Add all nominated production blocks (assemblers, refineries, etc.)
    List<IMyProductionBlock> producers = new List<IMyProductionBlock>();
    GridTerminalSystem.GetBlocksOfType<IMyProductionBlock>(producers);
    foreach(var box in producers)
        if(box.CubeGrid == Me.CubeGrid && isBlockFlagged(box, inputFlag))
            itemProducers.Add(box);
}

// Manually compares items by ID, because Prototech Scrap gets overlooked otherwise for some reason
public static int getItemAmount(IMyInventory inv, MyItemType item)
{
    int tally = 0;
    int index = 0;
    while(index < inv.ItemCount)
    {
        MyInventoryItem? slot = inv.GetItemAt(index);
        MyInventoryItem contents = slot.Value;
        
        if(contents.Type.SubtypeId.Contains(item.SubtypeId))
            tally += contents.Amount.ToIntSafe();
        index++;
    }
    return tally;
}

public static int getTotalItems(IMyInventory inv)
{
    int tally = 0;
    int index = 0;
    while(index < inv.ItemCount)
        tally += inv.GetItemAt(index++).Value.Amount.ToIntSafe();
    return tally;
}

// Returns true if any line of the given block's CustomData matches the given flag string
public static bool isBlockFlagged(IMyTerminalBlock block, string flagIn)
{
    string flags = block.CustomData.ToLower();
    string target = flagIn.ToLower();
    
    // Single line
    if(flags == target)
        return true;
    
    // Multi-line
    foreach(string f in flags.Split('\n'))
        if(f == target)
            return true;
    
    return false;
}
