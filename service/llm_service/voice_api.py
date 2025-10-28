"""
음성 시스템 API 애플리케이션
"""
from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from contextlib import asynccontextmanager
import logging
import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.append(str(Path(__file__).parent.parent))
sys.path.append(str(Path(__file__).parent))


from config import settings
#from models.schemas import HealthResponse, ComponentStatus
from routers import stt, nlu, tts, pipeline, live_stt
from services.voice_service import voice_service
from API.utils.error_handlers import (
    VoiceAPIException,
    voice_api_exception_handler,
    general_exception_handler,
    http_exception_handler
)
from fastapi.exceptions import HTTPException

# 로깅 설정
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)



@asynccontextmanager
async def lifespan(app: FastAPI):
    """애플리케이션 수명주기 관리"""
    # 시작
    logger.info("Starting Dobi Voice API...")
    try:
        # VoiceService 초기화는 자동으로 수행됨 (싱글톤)
        if voice_service.is_ready():
            logger.info("VoiceService initialized successfully")
        else:
            logger.warning("VoiceService initialization failed")
    except Exception as e:
        logger.error(f"Failed to initialize VoiceService: {e}")

    yield

    # 종료
    logger.info("Shutting down Dobi Voice API...")
    voice_service.shutdown()



app = FastAPI(
    title=settings.API_TITLE,
    version=settings.API_VERSION,
    description="""
    도비 음성 시스템 API

    ## 사용 예시

    ```python
    import requests

    # STT
    with open("audio.wav", "rb") as f:
        response = requests.post(
            "http://localhost:8000/api/v1/stt",
            files={"audio_file": f}
        )

    # Pipeline
    with open("audio.wav", "rb") as f:
        response = requests.post(
            "http://localhost:8000/api/v1/pipeline",
            files={"audio_file": f},
            data={"return_audio": "true"}
        )
    ```
    """,
    lifespan=lifespan,
    docs_url="/docs",
    redoc_url="/redoc"
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.CORS_ORIGINS,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.add_exception_handler(VoiceAPIException, voice_api_exception_handler)
app.add_exception_handler(HTTPException, http_exception_handler)
app.add_exception_handler(Exception, general_exception_handler)


app.include_router(stt.router, prefix=settings.API_PREFIX)
app.include_router(live_stt.router, prefix=settings.API_PREFIX)
app.include_router(nlu.router, prefix=settings.API_PREFIX)
app.include_router(tts.router, prefix=settings.API_PREFIX)
app.include_router(pipeline.router, prefix=settings.API_PREFIX)





@app.get("/", tags=["Root"])
async def root():
    """API 루트"""
    return {
        "message": "도비 음성 시스템 API",
        "version": settings.API_VERSION,
        "docs": "/docs",
        "health": f"{settings.API_PREFIX}/health"
    }


# 개발 서버 실행


@app.get(settings.API_PREFIX + "/health", tags=["Health"])
async def health():
    """간단한 헬스체크 엔드포인트: /api/v1/health"""
    # You can expand this to check downstream components if needed
    return {
        "success": True,
        "status": "ok",
        "version": settings.API_VERSION
    }


if __name__ == "__main__":
    import uvicorn

    logger.info(f"Starting server at {settings.HOST}:{settings.PORT}")
    uvicorn.run(
        "voice_api:app",
        host=settings.HOST,
        port=settings.PORT,
        reload=True,
        log_level="info"
    )
