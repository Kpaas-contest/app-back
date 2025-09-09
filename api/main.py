from fastapi import FastAPI
import os

app = FastAPI()

@app.get("/healthz")
def healthz():
    return {"ok": True, "service": os.getenv("CF_INSTANCE_GUID", "local")}
