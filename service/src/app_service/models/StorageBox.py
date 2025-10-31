from sqlalchemy import JSON, Column, Integer,  DateTime, Enum as SQLEnum
from enum import Enum
from sqlalchemy.sql import func
from database import Base

class StatusEnum(str, Enum):
    사용중 = '사용중'
    비움 = '비움'
    예약됨 = '예약됨'

class StorageBox(Base):
    __tablename__ = 'StorageBox'
    Storage_ID = Column(Integer, primary_key=True, autoincrement=True)
    MemberID = Column(Integer, nullable=False)
    LOC_ID = Column(Integer, nullable=False)
    Status = Column(SQLEnum(StatusEnum, native_enum=False),default=StatusEnum.사용중, nullable=False)
    CRE_DAT = Column(DateTime, server_default=func.now())
    CHG_DAT = Column(DateTime, server_default=func.now(), onupdate=func.now())
    Books = Column(JSON, nullable=True)
