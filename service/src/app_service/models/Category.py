from sqlalchemy import Column, String, DateTime
from sqlalchemy.sql import func
from database import Base

class Category(Base):
    __tablename__ = 'Category'
    CategoryID = Column(String(10), primary_key=True)
    ISBN = Column(String(13), primary_key=True)
    ParentCategoryID = Column(String(10), nullable=True)
    CRE_DAT = Column(DateTime, server_default=func.now())
    CHG_DAT = Column(DateTime, server_default=func.now(), onupdate=func.now())
