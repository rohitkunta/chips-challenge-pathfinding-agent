# Chip's Challenge Pathfinding Agent

An autonomous planning agent built in Java for a Chip's Challenge-style puzzle environment. The agent collects all chips, strategically acquires colored keys, unlocks keyed doors, and navigates to the exit portal — with behavior viewable via a GUI pathfinding playback.

## Highlights

- **A\* Search over Expanded State Space** — Searches over a composite state of (position + key inventory) using a priority-queue frontier, Manhattan distance heuristic, and state-signature hashing to prune revisited states.
- **Game-Rule-Aware Successor Generation** — Encodes full game mechanics into neighbor expansion: picking up keys (added to inventory), consuming keys to unlock colored doors (removed from inventory), wall/water blocking, and goal-door gating until all chips are collected.
- **Adaptive Rerouting** — When the path to the nearest chip or goal is blocked by a locked door, the agent automatically reroutes to find the nearest missing key before resuming its primary objective.
- **GUI Playback** — Includes a visual simulation panel that renders the agent's pathfinding decisions step-by-step on the game map.

## Project Structure

```
src/edu/ncsu/csc411/ps06/
├── agent/
│   └── Robot.java            # A* planning agent
├── environment/
│   ├── Action.java           # Movement actions
│   ├── Environment.java      # Game environment / state
│   ├── Position.java         # Grid position
│   ├── Tile.java             # Tile representation
│   └── TileStatus.java       # Tile type enum (wall, key, door, etc.)
├── simulation/
│   ├── RunSimulation.java    # Headless simulation runner
│   └── VisualizeSimulation.java  # GUI playback
└── utils/
    ├── ConfigurationLoader.java  # Config file parser
    └── MapManager.java           # Map loading
```

## How It Works

1. **Chip Collection Phase** — The agent uses A\* to find the shortest path to the nearest uncollected chip. If the path is blocked by a locked door, it reroutes to acquire the missing key first.
2. **Key Acquisition** — When a required key is missing, the agent searches for the nearest available key of that color using A\* with inventory simulation.
3. **Goal Navigation** — Once all chips are collected, the agent plans a direct path to the goal portal, again rerouting for keys if doors block the way.

## Running

Configure the simulation via the files in `config/` and run:
- `RunSimulation.java` — Runs the agent headlessly and outputs results.
- `VisualizeSimulation.java` — Launches the GUI to watch the agent solve the map step-by-step.
