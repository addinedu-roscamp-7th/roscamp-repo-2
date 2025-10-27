from sqlalchemy import Column, Integer, String, Enum, DateTime, Boolean
from sqlalchemy.sql import func
from database import Base

class MapFormatEnum(str, Enum):
    PGM = 'PGM'
    YAML = 'YAML'

class Map(Base):
    __tablename__ = 'Map'
    MapID = Column(Integer, primary_key=True, autoincrement=True)
    MapName = Column(String(100), nullable=True)
    FilePath = Column(String(255), nullable=True)
    Format = Column(Enum(MapFormatEnum, native_enum=False), nullable=True)
    CreatedTime = Column(DateTime, server_default=func.now())
    IsActive = Column(Boolean, default=True)
    CRE_DAT = Column(DateTime, server_default=func.now())
    CHG_DAT = Column(DateTime, server_default=func.now(), onupdate=func.now())
