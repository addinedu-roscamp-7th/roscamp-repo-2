from sqlalchemy import Column, Integer, String, Enum, DateTime
from sqlalchemy.sql import func
from database import Base

class MediaTypeEnum(str, Enum):
    VIDEO = '영상'
    IMAGE = '이미지'

class EventMedia(Base):
    __tablename__ = 'EventMedia'
    MediaID = Column(Integer, primary_key=True, autoincrement=True)
    Log_ID = Column(Integer, nullable=False)
    MediaType = Column(Enum(MediaTypeEnum, native_enum=False), nullable=True)
    FilePath = Column(String(255), nullable=True)
    CRE_DAT = Column(DateTime, server_default=func.now())
    CHG_DAT = Column(DateTime, server_default=func.now(), onupdate=func.now())
