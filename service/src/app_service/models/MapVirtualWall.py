from sqlalchemy import Column, Integer, DECIMAL, DateTime
from sqlalchemy.sql import func
from database import Base

class MapVirtualWall(Base):
    __tablename__ = 'MapVirtualWall'
    WallID = Column(Integer, primary_key=True, autoincrement=True)
    MapID = Column(Integer, nullable=False)
    StartX = Column(DECIMAL(9,6), nullable=True)
    StartY = Column(DECIMAL(9,6), nullable=True)
    EndX = Column(DECIMAL(9,6), nullable=True)
    EndY = Column(DECIMAL(9,6), nullable=True)
    CRE_DAT = Column(DateTime, server_default=func.now())
    CHG_DAT = Column(DateTime, server_default=func.now(), onupdate=func.now())
