"""Rapidly-exploring Random Tree (RRT) Motion Planner.
100% Python Standard Library.
"""

import math
import random

class RRTPlanner:
    """Sampling-based motion planner for 2D robotic navigation with circular obstacles."""
    def __init__(self, bounds=(0, 100, 0, 100), step_size=5.0, goal_bias=0.1, max_iter=1000):
        self.x_min, self.x_max, self.y_min, self.y_max = bounds
        self.step_size = float(step_size)
        self.goal_bias = float(goal_bias)
        self.max_iter = int(max_iter)

    @staticmethod
    def distance(p1, p2):
        return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

    def is_collision_free(self, p1, p2, obstacles):
        """Check if straight line segment between p1 and p2 collides with obstacles (x, y, radius)."""
        dist = self.distance(p1, p2)
        steps = max(int(dist / 1.0), 2)
        for s in range(steps + 1):
            t = s / steps
            px = p1[0] + t * (p2[0] - p1[0])
            py = p1[1] + t * (p2[1] - p1[1])
            for ox, oy, r in obstacles:
                if math.hypot(px - ox, py - oy) <= r:
                    return False
        return True

    def plan(self, start, goal, obstacles, seed=42):
        """Find a collision-free path from start to goal. Returns list of (x, y) coordinates."""
        rng = random.Random(seed)
        tree = {0: {"pos": start, "parent": None}}
        
        for i in range(1, self.max_iter + 1):
            if rng.random() < self.goal_bias:
                sample = goal
            else:
                sample = (rng.uniform(self.x_min, self.x_max), rng.uniform(self.y_min, self.y_max))
                
            nearest_id = min(tree.keys(), key=lambda nid: self.distance(tree[nid]["pos"], sample))
            nearest_pos = tree[nearest_id]["pos"]
            
            dist = self.distance(nearest_pos, sample)
            if dist < 1e-6:
                continue
            step = min(self.step_size, dist)
            theta = math.atan2(sample[1] - nearest_pos[1], sample[0] - nearest_pos[0])
            new_pos = (nearest_pos[0] + step * math.cos(theta), nearest_pos[1] + step * math.sin(theta))
            
            if self.is_collision_free(nearest_pos, new_pos, obstacles):
                tree[i] = {"pos": new_pos, "parent": nearest_id}
                
                if self.distance(new_pos, goal) <= self.step_size:
                    if self.is_collision_free(new_pos, goal, obstacles):
                        tree[i + 1] = {"pos": goal, "parent": i}
                        path = []
                        curr = i + 1
                        while curr is not None:
                            path.append((round(tree[curr]["pos"][0], 3), round(tree[curr]["pos"][1], 3)))
                            curr = tree[curr]["parent"]
                        return list(reversed(path))
        return None
