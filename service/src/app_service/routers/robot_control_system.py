from fastapi import APIRouter, Depends, Query,HTTPException
from schemas.robot_control_system import *
from sqlalchemy.orm import Session
from database import SessionLocal
from crud import robot_control_system_CURD
from models.Location import Location

router = APIRouter(prefix="/app", tags=["robot"])

def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()


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
# ###
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
# @router.post('/storage', response_model=ResponseApp)
# def updateBokInfo(RequestData: BoxInfoUpdate, db: Session = Depends(get_db)):
# DB 관련 테이블 없어서 상의해야함.