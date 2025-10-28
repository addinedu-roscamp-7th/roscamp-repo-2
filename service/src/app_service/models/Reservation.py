from sqlalchemy import Column, Integer, String, DateTime, Enum
from sqlalchemy.sql import func
from database import Base

class ReservationStatusEnum(str, Enum):
    WAIT = '대기'
    AVAILABLE = '가능'
    CANCELLED = '취소'

class Reservation(Base):
    __tablename__ = 'Reservation'
    ReservationID = Column(Integer, primary_key=True, autoincrement=True)
    ISBN = Column(String(13), nullable=False)
    MemberID = Column(Integer, nullable=False)
    ReservationTimestamp = Column(DateTime, server_default=func.now())
    Status = Column(Enum(ReservationStatusEnum, native_enum=False), default='대기', nullable=True)
    CRE_DAT = Column(DateTime, server_default=func.now())
    CHG_DAT = Column(DateTime, server_default=func.now(), onupdate=func.now())
