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

    // ---- the assembly hall --------------------------------------------------

    namespace
    {
        void copyName(char* destination, usize capacity, std::string_view name)
        {
            const usize count = std::min(name.size(), capacity - 1);
            for (usize i = 0; i < count; ++i)
            {
                destination[i] = name[i];
            }
            for (usize i = count; i < capacity; ++i)
            {
                destination[i] = '\0';
            }
        }
    } // namespace

    f64 assemblyProgress(const AssemblyComponent& assembly)
    {
        const f64 needed = assembly.ironNeededKg + assembly.copperNeededKg;
        if (needed <= 0.0)
        {
            return 0.0;
        }
        return std::clamp((assembly.ironPaidKg + assembly.copperPaidKg) / needed, 0.0,
                          1.0);
    }

    void assemblyOrder(AssemblyComponent& assembly, std::string_view name, f64 ironKg,
                       f64 copperKg)
    {
        copyName(assembly.blueprint, AssemblyComponent::kNameChars, name);
        assembly.ironNeededKg = std::max(0.0, ironKg);
        assembly.copperNeededKg = std::max(0.0, copperKg);
        // Changing the order throws away what is on the slipway. That is the
        // honest cost of changing your mind: metal already worked into a
        // different airframe does not come back.
        assembly.ironPaidKg = 0.0;
        assembly.copperPaidKg = 0.0;
        assembly.state = RecipeStateComponent::kIdle;
    }

    bool vehicleQueuePush(VehicleQueueComponent& queue, std::string_view name)
    {
        if (queue.count >= kVehicleQueueSlots)
        {
            return false;
        }
        copyName(queue.names[queue.count], AssemblyComponent::kNameChars, name);
        ++queue.count;
        return true;
    }

    std::string_view vehicleQueueFront(const VehicleQueueComponent& queue)
    {
        if (queue.count == 0)
        {
            return {};
        }
        return std::string_view(queue.names[0]);
    }

    void vehicleQueuePop(VehicleQueueComponent& queue)
    {
        if (queue.count == 0)
        {
            return;
        }
        for (u32 i = 1; i < queue.count; ++i)
        {
            copyName(queue.names[i - 1], AssemblyComponent::kNameChars,
                     std::string_view(queue.names[i]));
        }
        --queue.count;
        copyName(queue.names[queue.count], AssemblyComponent::kNameChars, {});
    }
} // namespace sw::factory
