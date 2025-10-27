from sqlalchemy import Column, String, DateTime
from sqlalchemy.sql import func
from database import Base

class SYS_CFG(Base):
    __tablename__ = 'SYS_CFG'
    CFG_KEY = Column(String(100), primary_key=True)
    CFG_V = Column(String(255), nullable=True)
    DES = Column(String(1000), nullable=True)
    CRE_DAT = Column(DateTime, server_default=func.now())
    CHG_DAT = Column(DateTime, server_default=func.now(), onupdate=func.now())
