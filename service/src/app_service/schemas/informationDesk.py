from pydantic import BaseModel
from typing import List, Optional
from enum import Enum
from datetime import datetime

#요청
class Authentication(BaseModel):
    cardNumber: str

class ISBN(BaseModel):
    isbn: str

class LOCID(BaseModel):
    locID: str

class SeatsReserv(BaseModel):
    locID: str
    startTime: datetime
    endTime: datetime



#응답
class AuthenticationResult(BaseModel):
    memberID: str
    name: str
    memberType: str

class LoanStatus(str, Enum):
    UNAVAILABLE = "대출불가"
    AVAILABLE = "대출가능"

class BookInfo(BaseModel):
    isbn: str
    title: str
    author: str
    publisher: str
    status: LoanStatus


class BookSearch(BaseModel):
    total: int
    page: int
    perPage: int
    result: List[BookInfo]

class BookLocation(BaseModel):
    locationName: str
    locType: str
    zoneName: str
    floor: str
    message: str | None = None

class Loan(BaseModel):
    loanDate: Optional[datetime] = None
    dueDate: Optional[datetime] = None
    locationName: Optional[str] | None = None #픽업 대출 경우 사용
    preparationTime: Optional[str] | None = None #픽업 대출 경우 사용

class ReserveResponse(BaseModel):
    reserveState: str

class GuidStatus(str, Enum):
    SUCCESS = "배정 완료"
    FAILURE = "배정 실패"

class Guid(BaseModel):
    status: GuidStatus
    estimatedTime: datetime



