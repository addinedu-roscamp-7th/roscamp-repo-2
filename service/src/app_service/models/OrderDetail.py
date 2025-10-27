from sqlalchemy import Column, Integer, DECIMAL, DateTime
from sqlalchemy.sql import func
from database import Base

class OrderDetail(Base):
    __tablename__ = 'OrderDetail'
    OrderDetailID = Column(Integer, primary_key=True, autoincrement=True)
    OrderID = Column(Integer, nullable=False)
    BeverageID = Column(Integer, nullable=False)
    Quantity = Column(Integer, nullable=False)
    PriceAtOrder = Column(DECIMAL(10,2), nullable=True)
    CRE_DAT = Column(DateTime, server_default=func.now())
    CHG_DAT = Column(DateTime, server_default=func.now(), onupdate=func.now())
