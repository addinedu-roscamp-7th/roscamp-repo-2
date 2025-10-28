from sqlalchemy import Column, Integer, String, Enum, DateTime
from database import Base
from sqlalchemy.sql import func
from enum import Enum as PyEnum

class AccountTypeEnum(str, PyEnum):
    MEMBER = "회원"
    STAFF = "직원"

class AccountStatusEnum(str, PyEnum):
    ACTIVE = "활성"
    INACTIVE = "정지"

class Account(Base):
    __tablename__ = "Account"

    ACC_ID = Column(Integer, primary_key=True, autoincrement=True)
    StaffID = Column(Integer, unique=True, nullable=True)
    MemberID = Column(Integer, unique=True, nullable=True)
    LoginID = Column(String(255), unique=True, nullable=False)
    Password = Column(String(255), nullable=False)
    AccountType = Column(Enum(AccountTypeEnum, native_enum=False), nullable=False)
    AccountStatus = Column(Enum(AccountStatusEnum, native_enum=False), default=AccountStatusEnum.ACTIVE.value, nullable=True)
    LastLoginTime = Column(DateTime, nullable=True)
    CRE_DAT = Column(DateTime, server_default=func.current_timestamp())
    CHG_DAT = Column(DateTime, server_default=func.current_timestamp(), onupdate=func.current_timestamp())
