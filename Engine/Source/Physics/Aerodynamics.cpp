#include "Physics/Aerodynamics.hpp"

#include "Core/FileSystem.hpp"
#include "Core/Json.hpp"
#include "Core/Log.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>

namespace sw::aero
{
    namespace
    {
        constexpr f64 kAdiabaticIndex = 1.4;    // diatomic air
        constexpr f64 kGasConstant = 287.053;   // J/(kg K), dry air
        constexpr f64 kLapseRateKPerM = 0.0065; // troposphere
        constexpr f64 kFloorTemperatureK = 180.0;

        /// The transonic rise, as (Mach, multiplier) knots. Shaped on the
        /// classic drag-divergence curve of a slender body: nothing happens
        /// until the local flow goes sonic somewhere on the nose, the wave
        /// drag then arrives all at once, and it slowly relaxes as the shock
        /// lies down against the body.
        constexpr f64 kMachKnots[][2] = {
            {0.00, 1.00}, {0.80, 1.00}, {0.95, 1.22}, {1.05, 1.50},
            {1.15, 1.60}, {1.40, 1.48}, {2.00, 1.30}, {3.00, 1.17},
            {5.00, 1.10}, {10.00, 1.08},
        };

        /// SIX FIGURES, AND NOT ONE MORE.
        ///
        /// The solver works in f32, so every digit past the seventh is
        /// noise — but printed at full double precision that noise triples
        /// the size of every table and makes a regenerated asset diff
        /// against itself. Rounding to six significant decimal digits makes
        /// the shortest round-trip form of the number those six digits, and
        /// anything under a ten-millionth is a rasterisation crumb rather
        /// than a force.
        [[nodiscard]] f64 quantise(f64 value)
        {
            if (!std::isfinite(value) || std::abs(value) < 1.0e-7)
            {
                return 0.0;
            }
            const f64 scale =
                std::pow(10.0, 5.0 - std::floor(std::log10(std::abs(value))));
            return std::round(value * scale) / scale;
        }

        [[nodiscard]] Vec3 perpendicular(const Vec3& axis)
        {
            const Vec3 reference =
                (std::abs(axis.y) < 0.9f) ? Vec3(0.0f, 1.0f, 0.0f) : Vec3(1.0f, 0.0f, 0.0f);
            return glm::normalize(glm::cross(reference, axis));
        }
    } // namespace

    // ------------------------------------------------------------------------
    // The air
    // ------------------------------------------------------------------------
    f64 density(const phys::AtmosphereComponent& atmosphere, f64 altitude)
    {
        if (altitude >= atmosphere.topAltitude)
        {
            return 0.0;
        }
        return atmosphere.surfaceDensity *
               std::exp(-std::max(altitude, 0.0) / atmosphere.scaleHeight);
    }

    f64 temperature(const phys::AtmosphereComponent& atmosphere, f64 altitude)
    {
        // Sea-level temperature is not stored on the component: it follows
        // from the two numbers that ARE, because an isothermal scale height
        // is H = R T / g and the density profile already committed to one.
        // Deriving it keeps a single source of truth — an atmosphere with a
        // 8.5 km scale height cannot also claim to be 400 K.
        const f64 seaLevelK = std::clamp(
            atmosphere.scaleHeight * 9.80665 / kGasConstant, kFloorTemperatureK, 900.0);
        return std::max(seaLevelK - kLapseRateKPerM * std::max(altitude, 0.0),
                        kFloorTemperatureK);
    }

    f64 speedOfSound(const phys::AtmosphereComponent& atmosphere, f64 altitude)
    {
        return std::sqrt(kAdiabaticIndex * kGasConstant * temperature(atmosphere, altitude));
    }

    f64 machDragFactor(f64 mach)
    {
        const f64 m = std::max(mach, 0.0);
        constexpr usize kCount = sizeof(kMachKnots) / sizeof(kMachKnots[0]);
        if (m >= kMachKnots[kCount - 1][0])
        {
            return kMachKnots[kCount - 1][1];
        }
        for (usize i = 1; i < kCount; ++i)
        {
            if (m <= kMachKnots[i][0])
            {
                const f64 span = kMachKnots[i][0] - kMachKnots[i - 1][0];
                const f64 t = (span > 0.0) ? (m - kMachKnots[i - 1][0]) / span : 0.0;
                return kMachKnots[i - 1][1] + t * (kMachKnots[i][1] - kMachKnots[i - 1][1]);
            }
        }
        return 1.0;
    }

    WorldVec3 windVelocity(const phys::AtmosphereComponent& atmosphere, f64 altitude,
                           const WorldVec3& up, const WorldVec3& east, f64 timeSeconds)
    {
        if (altitude < 0.0 || altitude >= atmosphere.topAltitude)
        {
            return WorldVec3{0.0};
        }
        // Two layers: a shallow surface breeze that vanishes at the ground
        // (a landed rocket must stay landed), and the jet stream — a broad
        // eastward river of air centred near three quarters of a scale
        // height up, which is where 11 km falls on Terra.
        const f64 jetCentre = 1.3 * atmosphere.scaleHeight;
        const f64 jetWidth = 0.8 * atmosphere.scaleHeight;
        const f64 offset = (altitude - jetCentre) / jetWidth;
        const f64 jet = 28.0 * std::exp(-offset * offset);
        // THE BOUNDARY LAYER, and it is not a detail: it is what keeps a
        // rocket on its pad. Air in contact with the ground is dragged to a
        // standstill by it, so the whole profile is multiplied by a factor
        // that vanishes at zero altitude. Without it a 2 m/s sea-level
        // breeze pushes on a standing vehicle every tick and the launch
        // complex slowly walks downwind.
        const f64 boundaryLayer = 1.0 - std::exp(-altitude / 250.0);
        const f64 speed = (5.0 + jet) * boundaryLayer *
                          std::exp(-std::max(0.0, altitude - jetCentre) /
                                   (2.0 * atmosphere.scaleHeight));

        // A slow veer, so a launch flown twice does not meet the same air.
        // Closed-form in time: no state to save, no divergence between the
        // predicted trajectory and the flown one.
        const f64 veer = 0.45 * std::sin(timeSeconds / 900.0) +
                         0.20 * std::sin(timeSeconds / 213.0);
        const WorldVec3 north = glm::cross(up, east);
        return (east * std::cos(veer) + north * std::sin(veer)) * speed;
    }

    // ------------------------------------------------------------------------
    // Sampling the table
    // ------------------------------------------------------------------------
    AeroSample sample(const AeroTable& table, const Vec3& flowDirectionLocal)
    {
        if (!table.valid())
        {
            return AeroSample{};
        }
        const Vec3 direction = glm::normalize(flowDirectionLocal);
        const f32 theta = std::acos(std::clamp(direction.z, -1.0f, 1.0f));
        f32 phi = std::atan2(direction.y, direction.x);
        if (phi < 0.0f)
        {
            phi += 2.0f * math::kPi;
        }

        const f32 thetaStep = math::kPi / static_cast<f32>(table.thetaCount - 1);
        const f32 phiStep = 2.0f * math::kPi / static_cast<f32>(table.phiCount);

        const f32 thetaCoord = std::clamp(theta / thetaStep, 0.0f,
                                          static_cast<f32>(table.thetaCount - 1));
        const u32 t0 = static_cast<u32>(thetaCoord);
        const u32 t1 = std::min(t0 + 1, table.thetaCount - 1);
        const f32 ft = thetaCoord - static_cast<f32>(t0);

        const f32 phiCoord = phi / phiStep;
        const u32 p0 = static_cast<u32>(phiCoord) % table.phiCount;
        const u32 p1 = (p0 + 1) % table.phiCount;
        const f32 fp = phiCoord - std::floor(phiCoord);

        const auto at = [&table](u32 t, u32 p) -> const AeroSample& {
            return table.samples[static_cast<usize>(t) * table.phiCount + p];
        };
        const AeroSample& s00 = at(t0, p0);
        const AeroSample& s01 = at(t0, p1);
        const AeroSample& s10 = at(t1, p0);
        const AeroSample& s11 = at(t1, p1);

        AeroSample out{};
        out.forceM2 = glm::mix(glm::mix(s00.forceM2, s01.forceM2, fp),
                               glm::mix(s10.forceM2, s11.forceM2, fp), ft);
        out.momentM3 = glm::mix(glm::mix(s00.momentM3, s01.momentM3, fp),
                                glm::mix(s10.momentM3, s11.momentM3, fp), ft);
        return out;
    }

    // ------------------------------------------------------------------------
    // .aero.json
    // ------------------------------------------------------------------------
    bool saveAeroTable(const AeroTable& table, const std::filesystem::path& path)
    {
        if (!table.valid())
        {
            SW_LOG_ERROR("Aero", "Refusing to save an invalid table for part {}",
                         table.partId);
            return false;
        }
        json::Value root = json::Value::makeObject();
        root.set("version", json::Value(1u));
        root.set("partId", json::Value(table.partId));
        root.set("part", json::Value(table.partName));
        root.set("thetaCount", json::Value(table.thetaCount));
        root.set("phiCount", json::Value(table.phiCount));
        root.set("maxAreaM2", json::Value(quantise(table.maxAreaM2)));
        root.set("referenceLengthM", json::Value(quantise(table.referenceLengthM)));

        // One JSON row per theta ring — the serializer keeps a scalar-only
        // array on one line, so the file is a readable grid of angles
        // instead of twelve thousand lines of one number each.
        json::Value rows = json::Value::makeArray();
        for (u32 t = 0; t < table.thetaCount; ++t)
        {
            json::Value row = json::Value::makeArray();
            for (u32 p = 0; p < table.phiCount; ++p)
            {
                const AeroSample& s = table.samples[static_cast<usize>(t) * table.phiCount + p];
                row.push(json::Value(quantise(s.forceM2.x)));
                row.push(json::Value(quantise(s.forceM2.y)));
                row.push(json::Value(quantise(s.forceM2.z)));
                row.push(json::Value(quantise(s.momentM3.x)));
                row.push(json::Value(quantise(s.momentM3.y)));
                row.push(json::Value(quantise(s.momentM3.z)));
            }
            rows.push(std::move(row));
        }
        root.set("rows", std::move(rows));

        std::ofstream out(path, std::ios::binary | std::ios::trunc);
        if (!out)
        {
            SW_LOG_ERROR("Aero", "Cannot write '{}'", path.string());
            return false;
        }
        out << json::serialize(root);
        return out.good();
    }

    bool loadAeroTable(const std::filesystem::path& path, AeroTable& out)
    {
        std::string text;
        try
        {
            const std::vector<u8> bytes = FileSystem::readBinaryFile(path);
            text.assign(bytes.begin(), bytes.end());
        }
        catch (...)
        {
            SW_LOG_ERROR("Aero", "Cannot read '{}'", path.string());
            return false;
        }

        std::string parseError;
        const json::Value root = json::parse(text, parseError);
        if (!parseError.empty() || !root.isObject())
        {
            SW_LOG_ERROR("Aero", "'{}': {}", path.string(),
                         parseError.empty() ? "not a JSON object" : parseError);
            return false;
        }

        AeroTable table{};
        table.partId = static_cast<u32>(root.number("partId", 0.0));
        table.partName = root.string("part");
        table.thetaCount = static_cast<u32>(root.number("thetaCount", 0.0));
        table.phiCount = static_cast<u32>(root.number("phiCount", 0.0));
        table.maxAreaM2 = root.number("maxAreaM2", 0.0);
        table.referenceLengthM = root.number("referenceLengthM", 1.0);

        const json::Value* rows = root.find("rows");
        if (table.partId == 0 || table.thetaCount < 2 || table.phiCount < 1 ||
            rows == nullptr || !rows->isArray() ||
            rows->asArray().size() != table.thetaCount)
        {
            SW_LOG_ERROR("Aero", "'{}': malformed header or row count", path.string());
            return false;
        }

        table.samples.resize(static_cast<usize>(table.thetaCount) * table.phiCount);
        for (u32 t = 0; t < table.thetaCount; ++t)
        {
            const json::Array& row = rows->asArray()[t].asArray();
            if (row.size() != static_cast<usize>(table.phiCount) * 6)
            {
                SW_LOG_ERROR("Aero", "'{}': row {} has {} numbers, expected {}",
                             path.string(), t, row.size(), table.phiCount * 6);
                return false;
            }
            for (u32 p = 0; p < table.phiCount; ++p)
            {
                const usize base = static_cast<usize>(p) * 6;
                AeroSample& s = table.samples[static_cast<usize>(t) * table.phiCount + p];
                s.forceM2 = Vec3(static_cast<f32>(row[base + 0].asNumber()),
                                 static_cast<f32>(row[base + 1].asNumber()),
                                 static_cast<f32>(row[base + 2].asNumber()));
                s.momentM3 = Vec3(static_cast<f32>(row[base + 3].asNumber()),
                                  static_cast<f32>(row[base + 4].asNumber()),
                                  static_cast<f32>(row[base + 5].asNumber()));
            }
        }
        out = std::move(table);
        return true;
    }

    std::vector<AeroTable> loadAeroTables(const std::filesystem::path& directory)
    {
        std::vector<AeroTable> tables;
        std::error_code error;
        if (!std::filesystem::is_directory(directory, error))
        {
            return tables;
        }
        std::vector<std::filesystem::path> files;
        for (const auto& entry : std::filesystem::directory_iterator(directory, error))
        {
            const std::filesystem::path& path = entry.path();
            if (path.extension() == ".json" &&
                path.stem().extension() == ".aero")
            {
                files.push_back(path);
            }
        }
        std::sort(files.begin(), files.end());
        for (const std::filesystem::path& path : files)
        {
            AeroTable table{};
            if (loadAeroTable(path, table))
            {
                tables.push_back(std::move(table));
            }
        }
        SW_LOG_INFO("Aero", "{} aerodynamic tables loaded from '{}'", tables.size(),
                    directory.string());
        return tables;
    }

    // ------------------------------------------------------------------------
    // Occlusion
    // ------------------------------------------------------------------------
    bool rayHitsBox(const OccluderBox& box, const Vec3& origin, const Vec3& direction,
                    f32 maxDistance, f32& outNear)
    {
        // Into the box's own frame: an oriented box is an axis-aligned one
        // seen from the right chair.
        const Quat inverse = glm::inverse(box.rotation);
        const Vec3 localOrigin = inverse * (origin - box.centre);
        const Vec3 localDirection = inverse * direction;

        f32 tMin = 0.0f;
        f32 tMax = maxDistance;
        for (int axis = 0; axis < 3; ++axis)
        {
            const f32 d = localDirection[axis];
            const f32 o = localOrigin[axis];
            const f32 h = box.halfExtents[axis];
            if (std::abs(d) < 1.0e-8f)
            {
                if (o < -h || o > h)
                {
                    return false; // parallel and outside the slab
                }
                continue;
            }
            f32 t0 = (-h - o) / d;
            f32 t1 = (h - o) / d;
            if (t0 > t1)
            {
                std::swap(t0, t1);
            }
            tMin = std::max(tMin, t0);
            tMax = std::min(tMax, t1);
            if (tMin > tMax)
            {
                return false;
            }
        }
        outNear = tMin;
        return true;
    }

    f32 exposure(std::span<const OccluderBox> boxes, u32 selfIndex,
                 const Vec3& flowDirectionVessel, f32 minimum)
    {
        if (selfIndex >= boxes.size())
        {
            return 1.0f;
        }
        const OccluderBox& self = boxes[selfIndex];
        const f32 flowLength = glm::length(flowDirectionVessel);
        if (flowLength < 1.0e-6f)
        {
            return 1.0f;
        }
        const Vec3 flow = flowDirectionVessel / flowLength;

        // How far upwind is "outside everything"? Far enough that no box
        // can start behind the ray's origin, which would let a part hide
        // inside its own neighbour.
        f32 reach = glm::length(self.halfExtents);
        for (const OccluderBox& box : boxes)
        {
            reach = std::max(reach, glm::length(box.centre - self.centre) +
                                        glm::length(box.halfExtents));
        }
        const f32 backOff = reach + 1.0f;

        // A 3x3 net stretched across the part's own frontal area. Sampling
        // the CENTRE alone would call a tank fully shielded the moment a
        // nose cone covered its middle; sampling the corners alone would
        // call it fully exposed the moment anything peeked past the edge.
        const Vec3 axisU = perpendicular(flow);
        const Vec3 axisV = glm::cross(flow, axisU);
        const auto extentAlong = [&self](const Vec3& axis) {
            const Vec3 x = self.rotation * Vec3(1.0f, 0.0f, 0.0f);
            const Vec3 y = self.rotation * Vec3(0.0f, 1.0f, 0.0f);
            const Vec3 z = self.rotation * Vec3(0.0f, 0.0f, 1.0f);
            return std::abs(glm::dot(axis, x)) * self.halfExtents.x +
                   std::abs(glm::dot(axis, y)) * self.halfExtents.y +
                   std::abs(glm::dot(axis, z)) * self.halfExtents.z;
        };
        const f32 halfU = extentAlong(axisU) * 0.62f;
        const f32 halfV = extentAlong(axisV) * 0.62f;

        u32 counted = 0;
        u32 clear = 0;
        for (int iu = -1; iu <= 1; ++iu)
        {
            for (int iv = -1; iv <= 1; ++iv)
            {
                const Vec3 origin = self.centre + axisU * (halfU * static_cast<f32>(iu)) +
                                    axisV * (halfV * static_cast<f32>(iv)) - flow * backOff;
                f32 selfNear = 0.0f;
                if (!rayHitsBox(self, origin, flow, backOff * 3.0f, selfNear))
                {
                    continue; // this sample misses the part itself: not its air
                }
                counted += 1;

                bool blocked = false;
                for (usize i = 0; i < boxes.size() && !blocked; ++i)
                {
                    if (boxes[i].ownerIndex == self.ownerIndex)
                    {
                        continue; // a part never shadows itself
                    }
                    // THE CHEAP NO, FIRST — and it has to be the RIGHT cheap
                    // no. This loop is the whole cost of the aerodynamics
                    // pass (every part against every other part's boxes,
                    // nine rays over), and it is quadratic in the part count,
                    // so a booster is where it hurts.
                    //
                    // The obvious rejection — is the box far from the ray's
                    // LINE — is nearly worthless here, and measurably so: on
                    // a rocket every box hugs the same axis the airflow runs
                    // along, so almost nothing is off-line and it bought 11 %
                    // on a 31-part vehicle. The rejection that pays is ALONG
                    // the flow: a box the ray reaches only AFTER it has
                    // already struck this part cannot possibly shadow it, and
                    // flying nose-first that is half the vehicle. Both are
                    // kept, cheapest first, because together they are still
                    // a handful of flops against a six-plane slab test.
                    const Vec3 toBox = boxes[i].centre - origin;
                    const f32 along = glm::dot(toBox, flow);
                    const f32 reachSquared =
                        glm::dot(boxes[i].halfExtents, boxes[i].halfExtents);
                    const f32 reach = std::sqrt(reachSquared);
                    if (along - reach > selfNear)
                    {
                        continue; // downstream of the part it would have to shade
                    }
                    const Vec3 perpendicular = toBox - flow * along;
                    if (glm::dot(perpendicular, perpendicular) > reachSquared)
                    {
                        continue; // the ray passes wide of this box's sphere
                    }
                    f32 otherNear = 0.0f;
                    if (rayHitsBox(boxes[i], origin, flow, selfNear, otherNear) &&
                        otherNear < selfNear - 1.0e-3f)
                    {
                        blocked = true;
                    }
                }
                clear += blocked ? 0u : 1u;
            }
        }
        if (counted == 0)
        {
            return 1.0f;
        }
        return std::max(minimum, static_cast<f32>(clear) / static_cast<f32>(counted));
    }

    // ------------------------------------------------------------------------
    // Damping
    // ------------------------------------------------------------------------
    Vec3 dampingMoment(f64 densityKgM3, f64 speedMps, f64 areaM2, f64 leverArmM,
                       const Vec3& angularVelocity)
    {
        if (speedMps < 1.0e-3 || densityKgM3 <= 0.0 || areaM2 <= 0.0)
        {
            return Vec3(0.0f);
        }
        const f64 coefficient =
            0.5 * densityKgM3 * speedMps * areaM2 * leverArmM * leverArmM;
        return Vec3(-coefficient * static_cast<f64>(angularVelocity.x),
                    -coefficient * static_cast<f64>(angularVelocity.y),
                    -coefficient * static_cast<f64>(angularVelocity.z));
    }
} // namespace sw::aero
