# infodesk_router.py
from fastapi import APIRouter, Depends
from sqlalchemy.orm import Session
from sqlalchemy import text
from database import SessionLocal

router = APIRouter(prefix="/app", tags=["Infodesk"])

# DB 세션 종속성
def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()

# 테이블 확인 엔드포인트
@router.get("/test-db")
def test_db(db: Session = Depends(get_db)):
    """
    DB에 존재하는 모든 테이블을 확인합니다.
    """
    try:
        result = db.execute(text("SHOW TABLES;"))
        tables = [row[0] for row in result]
        if not tables:
            return {"status": "OK", "tables": [], "message": "DB 연결 성공, 테이블 없음"}
        return {"status": "OK", "tables": tables}
    except Exception as e:
        return {"status": "FAIL", "error": str(e)}
