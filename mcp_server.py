"""MCP stdio server for RRT Motion Planner."""
import sys
import json

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

from client import RRTPlanner

def handle_rpc(request):
    req_id = request.get("id")
    method = request.get("method")
    params = request.get("params", {})

    if method == "tools/list":
        return {
            "jsonrpc": "2.0",
            "id": req_id,
            "result": {
                "tools": [
                    {
                        "name": "plan_rrt_path",
                        "description": "Plan collision-free path between start and goal avoiding obstacles",
                        "inputSchema": {
                            "type": "object",
                            "properties": {
                                "start": {"type": "array", "items": {"type": "number"}, "minItems": 2, "maxItems": 2},
                                "goal": {"type": "array", "items": {"type": "number"}, "minItems": 2, "maxItems": 2},
                                "obstacles": {
                                    "type": "array",
                                    "items": {
                                        "type": "array",
                                        "items": {"type": "number"},
                                        "minItems": 3,
                                        "maxItems": 3
                                    },
                                    "description": "List of [x, y, radius] obstacle definitions"
                                },
                                "step_size": {"type": "number"},
                                "seed": {"type": "integer"}
                            },
                            "required": ["start", "goal"]
                        }
                    }
                ]
            }
        }
    elif method == "tools/call":
        name = params.get("name")
        args = params.get("arguments", {})
        if name == "plan_rrt_path":
            start = tuple(args.get("start"))
            goal = tuple(args.get("goal"))
            obstacles = [tuple(obs) for obs in args.get("obstacles", [])]
            step_size = float(args.get("step_size", 5.0))
            seed = int(args.get("seed", 42))
            planner = RRTPlanner(step_size=step_size)
            path = planner.plan(start, goal, obstacles, seed=seed)
            return {"jsonrpc": "2.0", "id": req_id, "result": {"path": path, "waypoints": len(path) if path else 0}}
        return {"jsonrpc": "2.0", "id": req_id, "error": {"code": -32601, "message": f"Method {name} not found"}}
    return {"jsonrpc": "2.0", "id": req_id, "error": {"code": -32600, "message": "Invalid request"}}

def main():
    for line in sys.stdin:
        if not line.strip():
            continue
        try:
            req = json.loads(line)
            res = handle_rpc(req)
            sys.stdout.write(json.dumps(res) + "\n")
            sys.stdout.flush()
        except Exception as e:
            sys.stdout.write(json.dumps({"jsonrpc": "2.0", "error": {"code": -32700, "message": str(e)}}) + "\n")
            sys.stdout.flush()

if __name__ == "__main__":
    main()
