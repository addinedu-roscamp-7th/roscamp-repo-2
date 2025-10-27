from sqlalchemy import Column, Integer, String, Date, DateTime, DECIMAL, ForeignKey
from sqlalchemy.sql import func
from database import Base

class Rental(Base):
    __tablename__ = "Rental"

    RentalID = Column(Integer, primary_key=True, index=True, autoincrement=True)
    StaffID = Column(Integer, nullable=True)
    CopyID = Column(Integer, nullable=False)
    MemberID = Column(Integer, nullable=False)
    LoanDate = Column(Date, nullable=False)
    DueDate = Column(Date, nullable=False)
    ReturnDate = Column(Date, nullable=True)
    ExtensionCount = Column(Integer, default=0)
    LateFee = Column(DECIMAL(10,2), default=0.00)
    CRE_DAT = Column(DateTime, server_default=func.now())
    CHG_DAT = Column(DateTime, server_default=func.now(), onupdate=func.now())
