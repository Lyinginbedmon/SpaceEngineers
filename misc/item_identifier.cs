public Program()
{
    Runtime.UpdateFrequency = UpdateFrequency.Once;
    
    Me.CustomName = "Item ID identifer";
    
    List<IMyCargoContainer> cargo = new List<IMyCargoContainer>();
    GridTerminalSystem.GetBlocksOfType<IMyCargoContainer>(cargo);
    foreach(var box in cargo)
        if(box.CubeGrid == Me.CubeGrid)
        {
            Echo("Scanning "+box.CustomName);
            IMyInventory inv = box.GetInventory();
            
            int occupied = inv.ItemCount;
			int slot = 0;
            while(slot < occupied)
            {
                MyInventoryItem? contents = inv.GetItemAt(slot);
                if(contents != null)
                {
                    MyInventoryItem item = contents.Value;
                    Echo("> Slot "+slot+": "+item.Type.SubtypeId);
                }
				slot++;
            }
            
            return;
        }
    
    Echo("No containers found to scan");
}
