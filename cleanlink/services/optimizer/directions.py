# cleanlink/services/optimizer/directions.py
from __future__ import annotations
import os, asyncio
from math import radians, sin, cos, asin, sqrt
from typing import Tuple
import httpx
from dotenv import load_dotenv

load_dotenv()

NAVER_CLIENT_ID = os.getenv("NAVER_CLIENT_ID")
NAVER_CLIENT_SECRET = os.getenv("NAVER_CLIENT_SECRET")
NAVER_USE = os.getenv("NAVER_USE", "false").lower() == "true" and NAVER_CLIENT_ID and NAVER_CLIENT_SECRET

def haversine_m(a: Tuple[float, float], b: Tuple[float, float]) -> int:
    lat1, lon1 = a; lat2, lon2 = b
    R = 6371.0
    dlat = radians(lat2 - lat1); dlon = radians(lon2 - lon1)
    lat1 = radians(lat1); lat2 = radians(lat2)
    h = sin(dlat/2)**2 + cos(lat1)*cos(lat2)*sin(dlon/2)**2
    return int(2 * R * asin(sqrt(h)) * 1000)

async def naver_distance_time(a: Tuple[float, float], b: Tuple[float, float]) -> Tuple[int, int]:
    """
    네이버 도로거리/시간 조회 (가능하면 사용, 실패 시 예외 → 호출부에서 폴백).
    * 참고: 드라이빙 엔드포인트 예시
      GET https://naveropenapi.apigw.ntruss.com/map-direction/v1/driving?start=lon,lat&goal=lon,lat&option=trafast
      응답에서 route.trafast[0].summary.distance(m), duration(ms)
    """
    headers = {
        "X-NCP-APIGW-API-KEY-ID": NAVER_CLIENT_ID,
        "X-NCP-APIGW-API-KEY": NAVER_CLIENT_SECRET,
    }
    # 주의: 네이버는 "경도,위도" 순서
    start = f"{a[1]},{a[0]}"
    goal  = f"{b[1]},{b[0]}"
    url = f"https://naveropenapi.apigw.ntruss.com/map-direction/v1/driving?start={start}&goal={goal}&option=trafast"
    async with httpx.AsyncClient(timeout=10) as client:
        r = await client.get(url, headers=headers)
        r.raise_for_status()
        data = r.json()
        # 요약값 파싱 (없으면 KeyError → except에서 폴백)
        s = data["route"]["trafast"][0]["summary"]
        distance_m = int(s["distance"])         # meters
        duration_ms = int(s["duration"])        # ms
        duration_min = max(1, duration_ms // 60000)
        return distance_m, duration_min

async def distance_time(a: Tuple[float, float], b: Tuple[float, float]) -> Tuple[int, int]:
    """네이버 사용 가능하면 네이버, 아니면 하버사인(보행 속도 환산)"""
    if NAVER_USE:
        try:
            return await naver_distance_time(a, b)
        except Exception:
            pass  # 폴백
    # 폴백: 하버사인 + 보행 4.5km/h
    d_m = haversine_m(a, b)
    walking_mpm = 4500/60.0
    return d_m, max(1, int(d_m / walking_mpm))
