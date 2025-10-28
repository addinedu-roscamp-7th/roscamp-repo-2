from sqlalchemy import Column, Integer, String, DECIMAL, Enum, Date, DateTime, JSON
from sqlalchemy.sql import func
from database import Base

class RobotTypeEnum(str, Enum):
    AUTONOMOUS = '자율주행'
    BEVERAGE = '음료제조'

class RobotStatusEnum(str, Enum):
    IDLE = '대기'
    WORKING = '작업중'
    MAINTENANCE = '정비'
    CHARGING = '충전중'
    ERROR = '오류'

class Robot(Base):
    __tablename__ = 'Robot'
    RobotID = Column(Integer, primary_key=True, autoincrement=True)
    RobotName = Column(String(50), nullable=True)
    Model = Column(String(50), nullable=True)
    SerialNumber = Column(String(100), unique=True, nullable=True)
    RobotType = Column(Enum(RobotTypeEnum, native_enum=False), nullable=False)
    Status = Column(Enum(RobotStatusEnum, native_enum=False), default='대기', nullable=True)
    CurrentLOC_ID = Column(Integer, nullable=True)
    BatteryLevel = Column(DECIMAL(5,2), nullable=True)
    LastMaintenanceDate = Column(Date, nullable=True)
    TotalOperationHours = Column(Integer, default=0)
    Features = Column(JSON, nullable=True)
    CRE_DAT = Column(DateTime, server_default=func.now())
    CHG_DAT = Column(DateTime, server_default=func.now(), onupdate=func.now())
