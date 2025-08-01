#ifndef LA3DM_STATE_H
#define LA3DM_STATE_H

namespace la3dm {

    /// Occupancy state: before pruning: FREE, OCCUPIED, UNKNOWN, UNCERTAIN; after pruning: PRUNED
    enum class State : char {
        FREE,
        OCCUPIED,
        UNKNOWN,
        UNCERTAIN,
        PRUNED
    };

}

#endif // LA3DM_STATE_H