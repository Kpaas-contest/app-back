# cleanlink/services/optimizer/app.py
from fastapi import FastAPI
from pydantic import BaseModel, Field
from typing import List, Optional, Tuple
from math import radians, sin, cos, asin, sqrt
from datetime import datetime, timedelta
from ortools.constraint_solver import routing_enums_pb2, pywrapcp
from cleanlink.services.optimizer.directions import distance_time, haversine_m
import asyncio

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

    # 입력 정리
    v = req.vehicles[0]  # MVP: 차량 1대
    depot = (v.depot_lat, v.depot_lng)
    points = [depot] + [(j.lat, j.lng) for j in req.jobs]
    n = len(points)

    # --- 거리/시간 행렬 생성 (네이버 사용 가능 시 네이버, 아니면 폴백) ---
    dist_m = [[0] * n for _ in range(n)]        # meters
    walk_min_mat = [[0] * n for _ in range(n)]  # minutes (ETA 계산용)

    async def fill(i, j):
        if i == j:
            return
        d, mins = await distance_time(points[i], points[j])
        dist_m[i][j] = d
        walk_min_mat[i][j] = mins

    async def build_matrix():
        sem = asyncio.Semaphore(8)  # 동시 호출 제한
        async def wrapped(i, j):
            async with sem:
                await fill(i, j)
        await asyncio.gather(*(
            wrapped(i, j)
            for i in range(n) for j in range(n) if i != j
        ))

    asyncio.run(build_matrix())

    # --- OR-Tools TSP ---
    manager = pywrapcp.RoutingIndexManager(n, 1, 0)  # 1 vehicle, depot=0
    routing = pywrapcp.RoutingModel(manager)

    def distance_cb(from_index, to_index):
        i = manager.IndexToNode(from_index)
        j = manager.IndexToNode(to_index)
        return dist_m[i][j]

    cb_idx = routing.RegisterTransitCallback(distance_cb)
    routing.SetArcCostEvaluatorOfAllVehicles(cb_idx)

    params = pywrapcp.DefaultRoutingSearchParameters()
    params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    params.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    params.time_limit.FromSeconds(3)

    solution = routing.SolveWithParameters(params)
    route_order = []
    if solution:
        idx = routing.Start(0)
        while not routing.IsEnd(idx):
            node = manager.IndexToNode(idx)
            if node != 0:
                route_order.append(node)  # 1..N (jobs index + 1)
            idx = solution.Value(routing.NextVar(idx))
    else:
        route_order = list(range(1, n))

    # --- ETA/요약 계산 ---
    current_ts = datetime.strptime(req.date + " 09:00", "%Y-%m-%d %H:%M")
    route_id = 9001
    seq = 1
    prev = 0
    km_total = 0.0
    min_total = 0
    route_stops = []

    for node in route_order:
        d_m = dist_m[prev][node]
        walk_min = walk_min_mat[prev][node]
        current_ts += timedelta(minutes=walk_min)

        job = req.jobs[node - 1]
        current_ts += timedelta(minutes=job.service_min)

        km_total += d_m / 1000.0
        min_total += walk_min + job.service_min

        route_stops.append({
            "route_id": route_id,
            "seq": seq,
            "job_id": job.id,
            "eta_ts": current_ts.strftime("%Y-%m-%dT%H:%M:%SZ"),
            "etc_min": job.service_min,
            "dump_visit": False
        })
        seq += 1
        prev = node

    routes = [{
        "id": route_id,
        "vehicle_id": v.id,
        "distance_km": round(km_total, 2),
        "duration_min": min_total,
        "score": round(1.0 / (1.0 + km_total), 2)
    }]

    return {"routes": routes, "route_stops": route_stops}
