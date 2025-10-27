from sqlalchemy import Column, Integer, String, Enum, Date, DateTime
from sqlalchemy.sql import func
from database import Base

class PermissionLevelEnum(str, Enum):
    ADMIN = '관리자'
    OPERATOR = '운영자'

class Staff(Base):
    __tablename__ = 'Staff'
    StaffID = Column(Integer, primary_key=True, autoincrement=True)
    Name = Column(String(50), nullable=False)
    Position = Column(String(50), nullable=True)
    Contact = Column(String(20), nullable=True)
    HireDate = Column(Date, nullable=True)
    PermissionLevel = Column(Enum(PermissionLevelEnum, native_enum=False), default='운영자', nullable=True)
    CRE_DAT = Column(DateTime, server_default=func.now())
    CHG_DAT = Column(DateTime, server_default=func.now(), onupdate=func.now())
