"""Example demonstrating RRT path planning."""
from client import RRTPlanner

def main():
    planner = RRTPlanner(bounds=(0, 100, 0, 100), step_size=5.0, goal_bias=0.15)
    start = (10, 10)
    goal = (90, 90)
    obstacles = [(50, 50, 15), (30, 70, 10), (70, 30, 10)]
    
    print(f"Planning path from {start} to {goal} with {len(obstacles)} obstacles:")
    path = planner.plan(start, goal, obstacles, seed=42)
    if path:
        print(f"Found path with {len(path)} waypoints:")
        for idx, pt in enumerate(path):
            print(f"  [{idx:02d}] {pt}")
    else:
        print("Failed to find path within iteration limit.")

if __name__ == "__main__":
    main()
