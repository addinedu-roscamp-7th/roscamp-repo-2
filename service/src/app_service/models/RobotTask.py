from sqlalchemy import Column, Integer, String, DateTime, Enum, JSON
from sqlalchemy.sql import func
from database import Base

class RobotTaskTypeEnum(str, Enum):
    BOOK_PICKUP = '도서반출'
    BOOK_PLACE = '도서비치'
    BOOK_RETURN = '반납도서정리'
    SEAT_GUIDANCE = '좌석안내'

class RobotTaskStatusEnum(str, Enum):
    WAITING = '대기'
    ASSIGNED = '할당됨'
    IN_PROGRESS = '진행중'
    COMPLETED = '완료'
    FAILED = '실패'
    CANCELLED = '취소'

class RobotTask(Base):
    __tablename__ = 'RobotTask'
    TaskID = Column(Integer, primary_key=True, autoincrement=True)
    MemberID = Column(Integer, nullable=True)
    RobotID = Column(Integer, nullable=False)
    TaskType = Column(Enum(RobotTaskTypeEnum, native_enum=False), nullable=False)
    Priority = Column(Integer, default=5)
    Status = Column(Enum(RobotTaskStatusEnum, native_enum=False), default='대기', nullable=True)
    AssignedTime = Column(DateTime, nullable=True)
    StartTime = Column(DateTime, nullable=True)
    CompletionTime = Column(DateTime, nullable=True)
    Parameters = Column(JSON, nullable=True)
    Result = Column(JSON, nullable=True)
    CRE_DAT = Column(DateTime, server_default=func.now())
    CHG_DAT = Column(DateTime, server_default=func.now(), onupdate=func.now())
