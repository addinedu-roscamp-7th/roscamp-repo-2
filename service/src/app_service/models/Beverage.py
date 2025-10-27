from sqlalchemy import Column, Integer, String, DECIMAL, Text,  DateTime, Enum as SQLEnum
from sqlalchemy.sql import func
from database import Base
from enum import Enum

class StockStatusEnum(str, Enum):
    판매중 = '판매중'
    품절 = '품절'

class Beverage(Base):
    __tablename__ = 'Beverage'
    BeverageID = Column(Integer, primary_key=True, autoincrement=True)
    BeverageName = Column(String(100), nullable=False)
    Price = Column(DECIMAL(10,2), nullable=True)
    Description = Column(Text, nullable=True)
    StockStatus = Column(SQLEnum(StockStatusEnum, native_enum=False), default=StockStatusEnum.판매중, nullable=True)
    CRE_DAT = Column(DateTime, server_default=func.now())
    CHG_DAT = Column(DateTime, server_default=func.now(), onupdate=func.now())