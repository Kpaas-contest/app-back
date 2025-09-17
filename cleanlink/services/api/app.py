# cleanlink/services/api/app.py
from fastapi import FastAPI, HTTPException, Body
from fastapi.middleware.cors import CORSMiddleware
import os, httpx
from dotenv import load_dotenv

load_dotenv()
OPTIMIZER_URL = os.getenv("OPTIMIZER_URL", "http://127.0.0.1:8001")

app = FastAPI(title="CleanLink API Gateway")

# CORS (개발용: 전체 허용 → 배포 시 도메인 제한 권장)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/api/healthz")
def healthz():
    return {"ok": True, "optimizer": OPTIMIZER_URL}

@app.post("/api/optimize")
async def api_optimize(payload: dict = Body(..., description="OptimizeRequest JSON")):
    """프런트에서 보낸 JSON을 Optimizer로 프록시 전달"""
    url = f"{OPTIMIZER_URL}/optimize"
    try:
        async with httpx.AsyncClient(timeout=20) as client:
            r = await client.post(url, json=payload)
            r.raise_for_status()
            return r.json()
    except httpx.HTTPError as e:
        raise HTTPException(status_code=502, detail=f"optimizer error: {e}")
