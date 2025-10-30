from pydantic import BaseModel
from typing import List
from datetime import datetime

#요청 데이터 

#도서 픽업 작업 생성 요청
class BookLoc(BaseModel):
    x: float
    y: float
    z: float
class StoragePickLoc(BaseModel):
    x: float
    y: float
    z: float
class StorageLoc(BaseModel):
    x: float
    y: float
    theta: float
class ShelfLoc(BaseModel):
    x: float
    y: float
    theta: float


class BooksPickupTask(BaseModel):
    taskName: str
    book_id:str
    storage_id: int
    book_pick_pose: BookLoc #책장의 책 위치
    storage_approach_location: StorageLoc #픽업대 이동
    storage_slot_pose: BookLoc #도비 보관함
    shelf_approach_location: ShelfLoc #책장 이동



#도서 정보 요청
class BookInfo(BaseModel):
    title: str
    author: str
    publisher: str

class Loc(BaseModel): #좌석 정리 요청 , 좌석 정리 요청, 길 안내 요청
    locationName: str
    locType: str | None=None
    zoneName: str | None=None
    coordinateX: float
    coordinateY: float



#보관함 정보 업데이트
class MemberInfo(BaseModel):
    memberID: int
    name: str
    memberType: str

class BoxInfoUpdate(BaseModel):
    bookInfo: List[BookInfo]
    memberInfo: List[MemberInfo]
    storage: str

#도서 정보 업데이트
class BookInfoUpdate(BaseModel):
    lastScanTime: datetime

#메뉴 제조 요청
class Menu(BaseModel):
    beverageName: str
    quantity: int


class MenuRequest(BaseModel):
    orderID: int
    orderDetail: List[Menu]








#응답 데이터

#요청
class ResponseApp(BaseModel):
    message: str

#도서 정보 요청
class BooksInfoResponse(BaseModel):
    bookInfo: List[BookInfo]
    location: Loc

#길 안내 작업 요청
class GuideResponse(BaseModel):
    estimatedTime: int
    message: str

#메뉴 제조 요청
class MenuResponse(BaseModel):
    orderID: int
    message: str

