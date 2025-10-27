from sqlalchemy import Column, Integer, String, DateTime, Text, Enum
from sqlalchemy.sql import func
from database import Base

class StorageStatusEnum(str, Enum):
    STORAGE = '보관중'
    CLAIMED = '수령완료'

class LostAndFound(Base):
    __tablename__ = 'LostAndFound'
    LostItemID = Column(Integer, primary_key=True, autoincrement=True)
    FoundLOC_ID = Column(Integer, nullable=True)
    FoundTimestamp = Column(DateTime, server_default=func.now())
    ItemDescription = Column(Text, nullable=True)
    StorageStatus = Column(Enum(StorageStatusEnum, native_enum=False), default='보관중', nullable=True)
    ClaimantInfo = Column(String(100), nullable=True)
    CRE_DAT = Column(DateTime, server_default=func.now())
    CHG_DAT = Column(DateTime, server_default=func.now(), onupdate=func.now())
