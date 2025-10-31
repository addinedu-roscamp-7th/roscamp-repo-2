import json
from fastapi import APIRouter, Depends, Query,HTTPException
from models.StorageBox import StorageBox
from schemas.robot_control_system import *
from sqlalchemy.orm import Session
from database import SessionLocal
from crud import robot_control_system_CURD

import requests
from crud import robot_control_system_CURD as rcsc
from schemas import robot_control_system as rcs

router = APIRouter(prefix="/app", tags=["robot"])

def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()

#도서 픽업 작업 생성 요청
def book_pickup_request(request: rcs.BooksPickupTask):  # 도서 픽업 작업 생성 요청
    url = "http://192.168.0.131:8001/robot/pickup"  # RCS 서버 주소

    # 요청 보내기
    try:
        resp = requests.post(url, json=request.model_dump())
        print("Status code:", resp.status_code)
        print("Response:", resp.text)
    except requests.RequestException as e:
        print("Request failed:", e)

# 도서 정보 요청
@router.get("/books/{ISBN}", response_model=BooksInfoResponse)
def getBooksInfo(ISBN: str, db: Session = Depends(get_db)):
    # 책 조회
    books = robot_control_system_CURD.get_books(ISBN, db)
    if not books:
        raise HTTPException(status_code=404, detail=f"ISBN {ISBN}의 도서를 찾을 수 없습니다.")

    # LOC_ID 조회
    book_mat = robot_control_system_CURD.get_locID(ISBN, db)
    if not book_mat or not book_mat.LOC_ID:
        raise HTTPException(status_code=404, detail=f"ISBN {ISBN}에 대한 위치 정보가 없습니다.")

    locid = book_mat.LOC_ID

    # Location 정보 조회
    location_info = robot_control_system_CURD.get_loc_info(locid, db)
    if not location_info:
        raise HTTPException(status_code=404, detail=f"LOC_ID {locid}에 대한 위치 정보가 없습니다.")

    
    _bookinfo_list = [
        BookInfo(
            title=books.Title,
            author=books.Author,
            publisher=books.Publisher
        )
    ]

    _location = Loc(
        locationName=location_info.LocationName,
        locType=location_info.LOC_TY,
        zoneName=location_info.ZoneName,
        coordinateX=location_info.CoordinateX,
        coordinateY=location_info.CoordinateY
    )

    return BooksInfoResponse(
        bookInfo=_bookinfo_list,
        location=_location
    )



### 도서 정보 업데이트 (반납 여부는 데이터가 해당하는 컬럼이 없으므로 알고리즘을 구현해야함.) 
# 대여에 있는 DueDate와 도서자료의 LastScan Time을 비교해야함.
# 마지막 스캔일이 반납일 보다 지났으면 반납을 연체  9/1 9/10  9/11 
# 마지막 스캔일이 반납일 보다 지나지 않았으면 반납 완료  9/1 9/10 9/8

@router.patch('/books/update',response_model=BooksInfoResponse)
def updateBooksInfo(Barcode: str, lastscan: datetime, db:Session = Depends(get_db)):
    
    _barcode = robot_control_system_CURD.get_barcode(Barcode, db)
    
    if not _barcode:
        raise HTTPException(status_code=404, detail=f"Barcode: {Barcode}는 존재하지 않습니다.")
    
    _bookInfo = robot_control_system_CURD.get_books(_barcode.ISBN, db)
    if not _bookInfo:
        raise HTTPException(status_code=404, detail=f"ISBN {_bookInfo.ISBN}의 도서를 찾을 수 없습니다.")
    
    update_status = robot_control_system_CURD.update_books_info(Barcode, lastscan, db)
    
    if not update_status:
        
        raise HTTPException(status_code=404, detail=f"lastScan 업데이트가 실패하였습니다.")
    
    location_info = robot_control_system_CURD.get_loc_info(_barcode.LOC_ID, db)
    if not location_info:
        raise HTTPException(status_code=404, detail=f"LOC_ID {_barcode.LOC_ID}에 대한 위치 정보가 없습니다.")

    _bookinfo_list = [
        BookInfo(
            title=_bookInfo.Title,
            author=_bookInfo.Author,
            publisher=_bookInfo.Publisher
        )
    ]
    
    _location = Loc(
        locationName=location_info.LocationName,
        locType=location_info.LOC_TY,
        zoneName=location_info.ZoneName,
        coordinateX=location_info.CoordinateX,
        coordinateY=location_info.CoordinateY
    )
    return BooksInfoResponse(
        bookInfo=_bookinfo_list,
        location=_location
    )


# 보관함 정보 업데이트
@router.post('/storage', response_model=ResponseApp)
def updateBokInfo(request: BoxInfoUpdate, db: Session = Depends(get_db)):
    books = request.bookInfo
    member = request.memberInfo
    storage = request.storage

    loc_id = rcsc.get_locid(storage, db)

    if not loc_id:
        return ResponseApp(
        message="보관함 정보 생성 실패"
    )


    new_box = StorageBox(
        MemberID =member[0].memberID, 
        LOC_ID = loc_id.LOC_ID, 
        Status = "예약됨", 
        Books = json.dumps([book.model_dump() for book in books])
    )
    rcsc.create_storage_box(new_box, db)
    return ResponseApp(
        message="보관함 정보 생성"
    )

#메뉴 제조 요청
def menu_preparation_request(request: rcs.MenuRequest):  # 도서 픽업 작업 생성 요청
    url = "http://192.168.0.131:8001/robot/order"  # RCS 서버 주소

    # 요청 보내기
    try:
        resp = requests.post(url, json=request.model_dump())
        print("Status code:", resp.status_code)
        print("Response:", resp.text)
    except requests.RequestException as e:
        print("Request failed:", e)