from sqlalchemy import Column, Integer, String, Date, TIMESTAMP, Text
from sqlalchemy import Enum as SQLEnum
from sqlalchemy.sql import func
from database import Base
from enum import Enum

class Membertype(str, Enum):
    일반 = "일반"
    VIP = "VIP"

class State(str, Enum):
    활성 = "활성"
    중단 = "중단"

class Member(Base):
    __tablename__ = "Member"
    MemberID = Column(Integer, primary_key=True, index=True)
    CardNumber = Column(String(20), unique=True, nullable=False)
    Name = Column(String(50), nullable=False)
    Contact = Column(String(20))
    Address = Column(Text)
    JoinDate = Column(Date, nullable=False)
    MemberType = Column(SQLEnum(Membertype, native_enum=False),default=Membertype.일반, nullable=False)
    Status = Column(SQLEnum(State, native_enum=False),default=State.활성, nullable=False)
    CRE_DAT = Column(TIMESTAMP, server_default=func.current_timestamp(), nullable=True)
    CHG_DAT = Column(
        TIMESTAMP,
        server_default=func.current_timestamp(),
        server_onupdate=func.current_timestamp(),
        nullable=True
    )