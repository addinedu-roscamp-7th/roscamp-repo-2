from sqlalchemy import Column, String, Date, DECIMAL, TIMESTAMP
from sqlalchemy.sql import func
from database import Base  # database.py에서 Base 정의

class BookInfo(Base):
    __tablename__ = "BookInfo"  # 테이블 이름

    ISBN = Column(String(13), primary_key=True, nullable=False)
    Title = Column(String(255), nullable=False)
    Author = Column(String(100), nullable=True)
    Publisher = Column(String(50), nullable=True)
    PublishDate = Column(Date, nullable=True)
    Price = Column(DECIMAL(10,2), nullable=True)
    PurchaseDate = Column(Date, nullable=True)
    CRE_DAT = Column(TIMESTAMP, server_default=func.current_timestamp(), nullable=True)
    CHG_DAT = Column(
        TIMESTAMP,
        server_default=func.current_timestamp(),
        server_onupdate=func.current_timestamp(),
        nullable=True
    )