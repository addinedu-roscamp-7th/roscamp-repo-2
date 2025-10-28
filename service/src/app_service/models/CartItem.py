from sqlalchemy import Column, Integer, DateTime
from sqlalchemy.sql import func
from database import Base

class CartItem(Base):
    __tablename__ = 'CartItem'
    CartItemID = Column(Integer, primary_key=True, autoincrement=True)
    MemberID = Column(Integer, nullable=False)
    BeverageID = Column(Integer, nullable=False)
    Quantity = Column(Integer, default=1, nullable=True)
    CRE_DAT = Column(DateTime, server_default=func.now())
    CHG_DAT = Column(DateTime, server_default=func.now(), onupdate=func.now())
