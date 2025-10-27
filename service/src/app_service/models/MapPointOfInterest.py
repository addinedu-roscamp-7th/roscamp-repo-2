from sqlalchemy import Column, Integer, String, DECIMAL, DateTime
from sqlalchemy.sql import func
from database import Base

class MapPointOfInterest(Base):
    __tablename__ = 'MapPointOfInterest'
    POIID = Column(Integer, primary_key=True, autoincrement=True)
    MapID = Column(Integer, nullable=False)
    POIName = Column(String(100), nullable=True)
    CoordinateX = Column(DECIMAL(9,6), nullable=True)
    CoordinateY = Column(DECIMAL(9,6), nullable=True)
    CRE_DAT = Column(DateTime, server_default=func.now())
    CHG_DAT = Column(DateTime, server_default=func.now(), onupdate=func.now())
