from sqlalchemy import Column, Integer, String, DECIMAL, TIMESTAMP, Enum as SQLEnum
from sqlalchemy.sql import func
from database import Base
from enum import Enum

# CurrentStatus Enum 정의
class LocationStatus(str, Enum):
    사용가능 = "사용가능"
    사용불가 = "사용불가"

class Location(Base):
    __tablename__ = "Location"  # 테이블 이름

    LOC_ID = Column(Integer, primary_key=True, autoincrement=True)
    LocationName = Column(String(100), nullable=True)
    LOC_TY = Column(String(50), nullable=False)
    ZoneName = Column(String(50), nullable=True)
    Floor = Column(String(20), nullable=True)
    CoordinateX = Column(DECIMAL(9,6), nullable=True)
    CoordinateY = Column(DECIMAL(9,6), nullable=True)
    CoordinateZ = Column(DECIMAL(9,6), nullable=True)
    Capacity = Column(Integer, nullable=True)
    CurrentStatus = Column(
        SQLEnum(LocationStatus, native_enum=False),
        default=LocationStatus.사용가능,
        nullable=True
    )
    CRE_DAT = Column(TIMESTAMP, server_default=func.current_timestamp(), nullable=True)
    CHG_DAT = Column(
        TIMESTAMP,
        server_default=func.current_timestamp(),
        server_onupdate=func.current_timestamp(),
        nullable=True
    )
