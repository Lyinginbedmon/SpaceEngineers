// == Shine, Bright Morning Light OS - Data readouts ==
const string version = "1.0";

List<IMyTextPanel> screens = new List<IMyTextPanel>();
List<IMyCargoContainer> cargos = new List<IMyCargoContainer>();
List<IMyBatteryBlock> batteries = new List<IMyBatteryBlock>();
IMyProjector projector;

private int ticksRunning;
const String spinning = "-\\|/";
private static MyItemType[] ammos = new MyItemType[]
    {
        MyItemType.MakeAmmo("MediumCalibreAmmo"),
        MyItemType.MakeAmmo("NATO_25x184mm")
    };

private bool showDamage = false;

public Program()
{
    Runtime.UpdateFrequency = UpdateFrequency.Update10;
    Me.CustomName = "SBML Screen Control";
    
    List<IMyTextPanel> gridScreens = new List<IMyTextPanel>();
    GridTerminalSystem.GetBlocksOfType<IMyTextPanel>(gridScreens);
    foreach(var screen in gridScreens)
        if(screen.CubeGrid == Me.CubeGrid && screen.CustomData.Length > 0)
        {
            String data = screen.CustomData.ToLower();
            if(data == "ammo" || data == "damage" || data == "cargo")
            {
                screen.CustomName = "SBML Data Panel";
                screen.ShowInTerminal = false;
                screen.ShowInToolbarConfig = false;
                screens.Add(screen);
            }
        }
    
    List<IMyCargoContainer> boxes = new List<IMyCargoContainer>();
    GridTerminalSystem.GetBlocksOfType<IMyCargoContainer>(boxes);
    foreach(var box in boxes)
        if(box.CubeGrid == Me.CubeGrid)
        {
            box.CustomName = "SBML Cargo "+cargos.Count;
            box.ShowInTerminal = false;
            box.ShowInToolbarConfig = false;
            cargos.Add(box);
        }
    
    List<IMyBatteryBlock> bats = new List<IMyBatteryBlock>();
    GridTerminalSystem.GetBlocksOfType<IMyBatteryBlock>(bats);
    foreach(var box in bats)
        if(box.CubeGrid == Me.CubeGrid)
        {
            box.CustomName = "SBML Battery "+batteries.Count;
            box.ShowInTerminal = false;
            box.ShowInToolbarConfig = false;
            batteries.Add(box);
        }
    
    List<IMyProjector> projectors = new List<IMyProjector>();
    GridTerminalSystem.GetBlocksOfType<IMyProjector>(projectors);
    foreach(var project in projectors)
        if(project.CubeGrid == Me.CubeGrid)
        {
            project.CustomName = "SBML Projector";
            projector = project;
            break;
        }
}

public void Main(string argument, UpdateType updateSource)
{
    ++ticksRunning;
    Echo("SBML Screen OS Version "+version+" "+getSpinning());
    Echo("Show Damaged Blocks: "+showDamage);
    
    if(argument.Length > 0)
    {
        if(argument.ToLower() == "show_damage")
            showDamage = true;
        else if(argument.ToLower() == "hide_damage")
            showDamage = false;
    }
    
    foreach(var screen in screens)
    {
        if(screen == null || !screen.IsFunctional) continue;
        
        String type = screen.CustomData.ToLower();
        if(type == "ammo")
            handleAmmo(screen);
        else if(type == "damage")
            handleDamage(screen);
        else if(type == "cargo")
            handleCargo(screen);
    }
}

public char getSpinning()
{
    return spinning[this.ticksRunning % spinning.Length];
}

private void handleAmmo(IMyTextPanel screen)
{
    screen.WriteText(" = Ammo Status = ", false);
    foreach(var item in ammos)
    {
        int tally = 0;
        foreach(var box in cargos)
            tally += box.GetInventory().GetItemAmount(item).ToIntSafe();
        screen.WriteText("\n > "+item.SubtypeId+": "+tally, true);
        screen.WriteText("\n"+percentToBar((float)tally / 100), true);
    }
}

private void handleDamage(IMyTextPanel screen)
{
    screen.WriteText(" = Damage Report = \n", false);
    if(projector != null && projector.IsFunctional)
    {
        int projArmour = projector.RemainingArmorBlocks;
        int projBlocks = projector.RemainingBlocks - projArmour;
        if(projArmour > 0 || projBlocks > 0)
        {
            screen.WriteText("\nFrom Projection:", true);
            if(projArmour > 0)
                screen.WriteText("\n > Missing Armour "+projArmour, true);
            if(projBlocks > 0)
                screen.WriteText("\n > Missing Blocks "+projBlocks, true);
            float totalBlocks = 1F - ((float)projector.RemainingBlocks / (float)projector.TotalBlocks);
            screen.WriteText("\n"+percentToBar(totalBlocks)+"\n", true);
        }
    }
    
    List<String> damages = new List<String>();
    List<String> inoperable = new List<String>();
    List<IMyTerminalBlock> blocks = new List<IMyTerminalBlock>();
    GridTerminalSystem.GetBlocksOfType<IMyTerminalBlock>(blocks);
    foreach(var block in blocks)
    {
        float damage = 1F - getBlockHealth(block);
        if(damage > 0F)
        {
            if(block.IsFunctional)
                damages.Add(" > "+Math.Round(damage,2)*100+"% "+block.CustomName);
            else
            inoperable.Add(" > "+block.CustomName);
            
            if(showDamage)
                block.ShowOnHUD = true;
        }
    }
    
    bool hasDamage = (damages.Count + inoperable.Count) > 0;
    screen.WriteText("\n"+(hasDamage ? "Damage Detected:" : "No Damage"), true);
    if(hasDamage)
    {
        if(damages.Count > 0)
        {
            screen.WriteText("\n > Damaged: ", true);
            foreach(String str in damages)
                screen.WriteText("\n  "+str, true);
        }
        if(inoperable.Count > 0)
        {
            screen.WriteText("\n > Inoperable: ", true);
            foreach(String str in inoperable)
                screen.WriteText("\n  "+str, true);
        }
    }
}

private void handleCargo(IMyTextPanel screen)
{
    float total = 0F;
    float charge = 0F;
    foreach(var battery in batteries)
    {
        if(battery == null || !battery.IsFunctional) continue;
        if(battery.ChargeMode != ChargeMode.Recharge)
            charge += battery.CurrentStoredPower;
        total += battery.MaxStoredPower;
    }
    float power = charge/total;
    screen.WriteText("Power "+Math.Round(power,2)*100+"%");
    screen.WriteText("\n"+percentToBar(power), true);
    screen.WriteText("\n", true);
    
    total = 0F;
    charge = 0F;
    foreach(var box in cargos)
    {
        if(box == null || !box.IsFunctional) continue;
        total += box.GetInventory().MaxVolume.ToIntSafe();
        charge += box.GetInventory().CurrentVolume.ToIntSafe();
    }
    float cargo = charge/total;
    screen.WriteText("Cargo "+Math.Round(cargo,2)*100+"%", true);
    screen.WriteText("\n"+percentToBar(cargo), true);
}

public string percentToBar(float percentage, float divisor)
{
    percentage = clamp(percentage, 0F, 1F);
    String bar = "[";
    float val = 0F;
    while(val < 1F)
    {
        bar += val <= percentage ? "|" : "'";
        val += divisor;
    }
    bar += "]";
    return bar;
}

public string percentToBar(float percentage)
{
    return percentToBar(percentage, 0.0125F);
}

public float clamp(float val, float min, float max)
{
    return Math.Max(Math.Min(min,max), Math.Min(Math.Max(min,max), val));
}

public float getBlockHealth(IMyTerminalBlock block)
{
    IMySlimBlock slimblock = block.CubeGrid.GetCubeBlock(block.Position);
    float MaxIntegrity = slimblock.MaxIntegrity;
    float BuildIntegrity = slimblock.BuildIntegrity;
    float CurrentDamage = slimblock.CurrentDamage;
    return (BuildIntegrity - CurrentDamage) / MaxIntegrity;
}