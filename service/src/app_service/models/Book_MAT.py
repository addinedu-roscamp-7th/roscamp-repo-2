from sqlalchemy import Column, Integer, String, DECIMAL, DateTime, Enum  as SQLEnum
from sqlalchemy.sql import func
from database import Base
from enum import Enum

class BookMATStatusEnum(str, Enum):
    신규 = '신규'
    좋음 = '좋음'
    손상 = '손상'

class Book_MAT(Base):
    __tablename__ = 'Book_MAT'
    CopyID = Column(Integer, primary_key=True, autoincrement=True)
    LOC_ID = Column(Integer, nullable=False)
    ISBN = Column(String(13), nullable=False)
    Barcode = Column(String(50), unique=True, nullable=True)
    RFIDTag = Column(String(100), unique=True, nullable=True)
    Status = Column(SQLEnum(BookMATStatusEnum, native_enum=False), default=BookMATStatusEnum.신규, nullable=True)
    LastScanTime = Column(DateTime, nullable=True)
    CRE_DAT = Column(DateTime, server_default=func.now())
    CHG_DAT = Column(DateTime, server_default=func.now(), onupdate=func.now())