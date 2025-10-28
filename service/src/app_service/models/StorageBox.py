from sqlalchemy import Column, Integer, String, Enum, DateTime, Boolean
from sqlalchemy.sql import func
from database import Base

class StatusEnum(str, Enum):
    사용중 = '사용중'
    비움 = '비움'
    예약됨 = '예약됨'

class Map(Base):
    __tablename__ = 'StorageBox'
    storage_ID = Column(Integer, primary_key=True, autoincrement=True)
    MemberID = Column(Integer, nullable=False)
    LOC_ID = Column(Integer, nullable=False)
    ISBN = Column(String, nullable=False)
    Status = Column(Enum(StatusEnum, native_enum=False),default=StatusEnum.사용중, nullable=False)
    cre_dat = Column(DateTime, server_default=func.now())
    CHG_DAT = Column(DateTime, server_default=func.now(), onupdate=func.now())
