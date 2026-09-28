# Rapidly-exploring Random Tree (RRT) Motion Planner Skill

Sampling-based 2D motion planner for autonomous robots navigating cluttered obstacle fields.

```mermaid
flowchart TD
    Sample["Sample Random Configuration (q_rand)"] --> Nearest["Find Nearest Node in Tree (q_near)"]
    Nearest --> Steer["Steer Toward Sample (q_new = q_near + step * u)"]
    Steer --> Collision{"Collision Free Segment?"}
    Collision -- Yes --> Add["Add q_new to Tree"]
    Collision -- No --> Sample
    Add --> GoalCheck{"Near Goal?"}
    GoalCheck -- Yes --> Reconstruct["Reconstruct Collision-Free Path"]
    GoalCheck -- No --> Sample
```

## Features
- **100% Python Standard Library**: Pure mathematical geometric primitives.
- **Goal Biasing**: Accelerated convergence toward target configuration.
- **Continuous Collision Checking**: Segment interpolation against circular obstacle geometries.
