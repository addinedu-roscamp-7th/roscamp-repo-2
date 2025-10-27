from sqlalchemy import Column, Integer, DateTime, DECIMAL,  JSON, Enum as SQLEnum
from sqlalchemy.sql import func
from database import Base
from enum import Enum

class OrderStatusEnum(str, Enum):
    접수 = '접수'
    제조중 = '제조중'
    완료 = '완료'
    픽업대기 = '픽업대기'

class BeverageOrder(Base):
    __tablename__ = 'BeverageOrder'
    OrderID = Column(Integer, primary_key=True, autoincrement=True)
    MemberID = Column(Integer, nullable=False)
    OrderTimestamp = Column(DateTime, server_default=func.now())
    TotalAmount = Column(DECIMAL(10,2), nullable=True)
    PaymentInfo = Column(JSON, nullable=True)
    OrderStatus = Column(SQLEnum(OrderStatusEnum, native_enum=False), default=OrderStatusEnum.접수, nullable=True)
    CRE_DAT = Column(DateTime, server_default=func.now())
    CHG_DAT = Column(DateTime, server_default=func.now(), onupdate=func.now())
