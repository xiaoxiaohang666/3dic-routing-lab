#pragma once

#include "parser/def_parser.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>
#include <unordered_map>

namespace eda::compat {

// These two details were not specified by the October 20 update. Defaults
// preserve the old scorer's first-layer pin and repeated-track conventions.
struct ResourceOptions {
    bool pin_first_layer{true};
    bool deduplicate_tracks{false};
    // Explicit observed 0.4.0 platform resource policy. This changes resources
    // only, not physical PIN geometry or October 20 legality/length rules.
    bool platform_20261024{false};
    // October 29 assumption, opt-in only: for each physical PIN identity,
    // actual layer and covered GCell, an empty union of that PIN's real tracks
    // contributes one virtual unit. Distinct PIN units add, including same-net
    // and overlapping PINs. OBS receives no minimum. Multiple RECTs of one PIN
    // do not multiply the floor. Published notes do not settle this granularity.
    bool pin_zero_track_floor{false};
    // Geometry observed to reproduce four 2026-11-04 platform overflow maps exactly
    // (see docs/rules/fixed-resource-model-20261105.md); opt-in, not a published rule.
    // Every shape uses half-open track intervals; a PIN clip holding no track adds the
    // previous global track index to the cell's union; fixed demand is capped at
    // capacity. Routes and TSVs are never capped. Replaces pin_zero_track_floor.
    bool observed_fixed_20261104{false};
};

// Resource model reconstructed from the published October 20 scoring notes.
// This is independent of the router's historical official-resource model.
// The DEF parser and its grid must outlive this object.
class Resources {
public:
    Resources(const DefParser& def, const LefParser& lef, ResourceOptions options = {});

    size_t size() const { return fixed_demand_.size(); }
    int node(int col, int row, int layer) const;
    int col(int id) const;
    int row(int id) const;
    int layer(int id) const;
    uint32_t capacity(int id) const;
    uint32_t fixed_demand(int id) const;
    const std::vector<uint32_t>& fixed_demands() const { return fixed_demand_; }
    uint32_t pin_zero_track_floor_demand(int id) const;
    uint64_t pin_zero_track_floor_total() const { return pin_floor_total_; }
    size_t pin_zero_track_floor_cells() const { return pin_floor_demand_.size(); }
    static const char* pin_zero_track_floor_model() {
        return "per-physical-pin-layer-cell-empty-track-union-additive";
    }

    // Sorted, unique cell-layer IDs covered by this net's actual pin shapes.
    // Historical profiles use the fixed-demand pin-layer policy. Platform
    // macro-PIN credit retains only original shapes on the first geometry
    // layer; IO credit retains every original layer. No whole-grid net masks.
    std::vector<int> pin_nodes(const Net& net) const;

    const GCellGrid& grid() const { return grid_; }
    const ResourceOptions& options() const { return options_; }

private:
    const GCellGrid& grid_;
    ResourceOptions options_;
    size_t plane_{0};
    std::array<Direction, 21> directions_{};
    std::array<std::vector<uint32_t>, 21> capacities_;
    std::vector<uint32_t> fixed_demand_;
    // Store only virtual zero-track PIN floors; old profiles allocate no mask.
    std::unordered_map<int,uint32_t> pin_floor_demand_;
    uint64_t pin_floor_total_{};

    void check_id(int id) const;
};

} // namespace eda::compat
