# cleanlink/services/optimizer/app.py
from fastapi import FastAPI
from pydantic import BaseModel, Field
from typing import List, Optional, Tuple
from math import radians, sin, cos, asin, sqrt
from datetime import datetime, timedelta
from ortools.constraint_solver import routing_enums_pb2, pywrapcp

class Vehicle(BaseModel):
    id: int
    type: str = "WALK"
    capacity: Optional[int] = 0
    depot_lat: float
    depot_lng: float

class Job(BaseModel):
    id: int
    name: str
    lat: float
    lng: float
    service_min: int
    priority: int
    tw_start: Optional[str] = None
    tw_end: Optional[str] = None
    date: str
    status: str

class OptimizeRequest(BaseModel):
    date: str = Field(..., example="2025-10-01")
    area: str = Field(..., example="SEOUL-XX")
    vehicles: List[Vehicle]
    jobs: List[Job]

def haversine_km(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    lat1, lon1 = a; lat2, lon2 = b
    R = 6371.0
    dlat = radians(lat2 - lat1); dlon = radians(lon2 - lon1)
    lat1 = radians(lat1); lat2 = radians(lat2)
    h = sin(dlat/2)**2 + cos(lat1)*cos(lat2)*sin(dlon/2)**2
    return 2 * R * asin(sqrt(h))

app = FastAPI(title="CleanLink Optimizer")

@app.get("/healthz")
def healthz():
    return {"ok": True}

@app.post("/optimize")
def optimize(req: OptimizeRequest):
    assert req.vehicles, "at least one vehicle required"
    v = req.vehicles[0]
    depot = (v.depot_lat, v.depot_lng)
    points = [depot] + [(j.lat, j.lng) for j in req.jobs]
    n = len(points)

    # distance matrix (meters)
    dist_m = [[0]*n for _ in range(n)]
    for i in range(n):
        for j in range(n):
            if i == j: continue
            dist_m[i][j] = int(haversine_km(points[i], points[j]) * 1000)

    manager = pywrapcp.RoutingIndexManager(n, 1, 0)  # 1 vehicle, depot=0
    routing = pywrapcp.RoutingModel(manager)

    def distance_cb(fi, ti):
        i = manager.IndexToNode(fi); j = manager.IndexToNode(ti)
        return dist_m[i][j]
    cb_idx = routing.RegisterTransitCallback(distance_cb)
    routing.SetArcCostEvaluatorOfAllVehicles(cb_idx)

    params = pywrapcp.DefaultRoutingSearchParameters()
    params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    params.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    params.time_limit.FromSeconds(3)

    sol = routing.SolveWithParameters(params)
    route_order = []
    if sol:
        idx = routing.Start(0)
        while not routing.IsEnd(idx):
            node = manager.IndexToNode(idx)
            if node != 0:
                route_order.append(node)  # 1..N = job index +1
            idx = sol.Value(routing.NextVar(idx))
    else:
        route_order = list(range(1, n))

    # Simple ETA/duration estimate (walking 4.5 km/h)
    walking_kmh = 4.5; walking_mpm = walking_kmh * 1000 / 60
    current_ts = datetime.strptime(req.date + " 09:00", "%Y-%m-%d %H:%M")
    route_id = 9001; seq = 1; prev = 0
    km_total = 0.0; min_total = 0
    route_stops = []

    for node in route_order:
        d_m = dist_m[prev][node]
        walk_min = int(d_m / walking_mpm)
        current_ts += timedelta(minutes=walk_min)
        job = req.jobs[node-1]
        current_ts += timedelta(minutes=job.service_min)
        km_total += d_m/1000.0; min_total += walk_min + job.service_min
        route_stops.append({
            "route_id": route_id, "seq": seq, "job_id": job.id,
            "eta_ts": current_ts.strftime("%Y-%m-%dT%H:%M:%SZ"),
            "etc_min": job.service_min, "dump_visit": False
        })
        seq += 1; prev = node

    routes = [{
        "id": route_id, "vehicle_id": v.id,
        "distance_km": round(km_total, 2),
        "duration_min": min_total,
        "score": round(1.0/(1.0+km_total), 2)
    }]
    return {"routes": routes, "route_stops": route_stops}
