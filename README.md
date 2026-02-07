• Built an autonomous planning agent in Java for a Chip’s Challenge–style puzzle environment that collects all chips,
strategically acquires colored keys, unlocks keyed doors, and navigates to the exit/portal, with behavior
viewable via a GUI pathfinding playback
• Implemented A* in Java over an expanded state (position + key inventory) using a priority-queue frontier,
Manhattan heuristic, and state hashing to prune repeats; encoded game rules in successor generation
(pickup/consume keys, door gating, blocked reroutes to nearest chip/key).
