#include "Factory/FactoryComponents.hpp"

#include <algorithm>

namespace sw::factory
{
    f64 inventoryVolume(const InventoryComponent& inventory)
    {
        f64 volume = 0.0;
        for (const InventorySlot& slot : inventory.slots)
        {
            if (slot.resource != res::Resource::Count)
            {
                volume += slot.units * res::definition(slot.resource).volumePerUnitM3;
            }
        }
        return volume;
    }

    f64 inventoryCount(const InventoryComponent& inventory, res::Resource resource)
    {
        f64 units = 0.0;
        for (const InventorySlot& slot : inventory.slots)
        {
            if (slot.resource == resource)
            {
                units += slot.units;
            }
        }
        return units;
    }

    f64 inventoryAdd(InventoryComponent& inventory, res::Resource resource, f64 units)
    {
        if (units <= 0.0 || resource == res::Resource::Count)
        {
            return 0.0;
        }

        // Volume bound: matter takes real space.
        const f64 volumePerUnit = res::definition(resource).volumePerUnitM3;
        const f64 freeVolume = inventory.volumeCapacityM3 - inventoryVolume(inventory);
        const f64 acceptable = std::min(units, std::max(0.0, freeVolume) / volumePerUnit);
        if (acceptable <= 0.0)
        {
            return 0.0;
        }

        // Prefer an existing stack of the same resource, then an empty slot.
        for (InventorySlot& slot : inventory.slots)
        {
            if (slot.resource == resource)
            {
                slot.units += acceptable;
                return acceptable;
            }
        }
        for (InventorySlot& slot : inventory.slots)
        {
            if (slot.resource == res::Resource::Count)
            {
                slot.resource = resource;
                slot.units = acceptable;
                return acceptable;
            }
        }
        return 0.0; // no free slot
    }

    f64 inventoryFreeUnits(const InventoryComponent& inventory, res::Resource resource)
    {
        if (resource == res::Resource::Count)
        {
            return 0.0;
        }
        const f64 volumePerUnit = res::definition(resource).volumePerUnitM3;
        const f64 freeVolume =
            std::max(0.0, inventory.volumeCapacityM3 - inventoryVolume(inventory));
        const f64 byVolume = freeVolume / volumePerUnit;

        // A slot is needed too: an inventory with free volume but no stack
        // of this resource and no empty slot cannot take a single unit.
        for (const InventorySlot& slot : inventory.slots)
        {
            if (slot.resource == resource || slot.resource == res::Resource::Count)
            {
                return byVolume;
            }
        }
        return 0.0;
    }

    f64 inventoryRemove(InventoryComponent& inventory, res::Resource resource, f64 units)
    {
        if (units <= 0.0)
        {
            return 0.0;
        }
        f64 remaining = units;
        for (InventorySlot& slot : inventory.slots)
        {
            if (slot.resource != resource || remaining <= 0.0)
            {
                continue;
            }
            const f64 taken = std::min(slot.units, remaining);
            slot.units -= taken;
            remaining -= taken;
            if (slot.units <= 1.0e-12)
            {
                slot.resource = res::Resource::Count;
                slot.units = 0.0;
            }
        }
        return units - remaining;
    }

    i32 linkAddChannel(ItemLinkComponent& link, ecs::Entity source,
                       res::Resource resource, f64 unitsPerSecond)
    {
        if (resource == res::Resource::Count)
        {
            return -1;
        }
        i32 free = -1;
        for (u32 i = 0; i < kMaxLinkChannels; ++i)
        {
            LinkChannel& channel = link.channels[i];
            if (channel.resource == resource && channel.source == source)
            {
                // The same feed, declared twice: raise the rate rather than
                // spend a channel on it. Two belts side by side ARE faster.
                channel.unitsPerSecond += unitsPerSecond;
                return static_cast<i32>(i);
            }
            if (free < 0 && channel.resource == res::Resource::Count)
            {
                free = static_cast<i32>(i);
            }
        }
        if (free < 0)
        {
            return -1;
        }
        link.channels[static_cast<u32>(free)] = {source, resource, unitsPerSecond, 0.0};
        return free;
    }

    f64 linkFlow(const ItemLinkComponent& link, res::Resource resource)
    {
        for (const LinkChannel& channel : link.channels)
        {
            if (channel.resource == resource)
            {
                return channel.flowUnitsPerSecond;
            }
        }
        return 0.0;
    }

    f64 linkFlowFrom(const ItemLinkComponent& link, ecs::Entity source)
    {
        f64 total = 0.0;
        for (const LinkChannel& channel : link.channels)
        {
            if (channel.source == source && channel.resource != res::Resource::Count)
            {
                total += channel.flowUnitsPerSecond;
            }
        }
        return total;
    }
} // namespace sw::factory
