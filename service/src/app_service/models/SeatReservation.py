from sqlalchemy import Column, Integer, DateTime
from sqlalchemy.sql import func
from database import Base

class SeatReservation(Base):
    __tablename__ = 'SeatReservation'
    SeatReservationID = Column(Integer, primary_key=True, autoincrement=True)
    LOC_ID = Column(Integer, nullable=False)
    MemberID = Column(Integer, nullable=False)
    StartTime = Column(DateTime, nullable=False)
    EndTime = Column(DateTime, nullable=False)
    CRE_DAT = Column(DateTime, server_default=func.now())
    CHG_DAT = Column(DateTime, server_default=func.now(), onupdate=func.now())
