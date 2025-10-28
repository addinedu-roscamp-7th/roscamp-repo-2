from sqlalchemy import Column, Integer, String, Enum, DateTime
from sqlalchemy.sql import func
from database import Base

class EventSeverityEnum(str, Enum):
    INFO = '정보'
    WARNING = '경고'
    CRITICAL = '심각'

class SYS_EVT_LOG(Base):
    __tablename__ = 'SYS_EVT_LOG'
    Log_ID = Column(Integer, primary_key=True, autoincrement=True)
    EVT_TY = Column(String(100), nullable=True)
    Severity = Column(Enum(EventSeverityEnum, native_enum=False), default='정보', nullable=True)
    OccurredTime = Column(DateTime, server_default=func.now())
    RelatedRobotID = Column(Integer, nullable=True)
    Details = Column(String(1000), nullable=True)
    CRE_DAT = Column(DateTime, server_default=func.now())
