from sqlalchemy.orm import Session
from models.BookInfo import BookInfo
from models.Book_MAT import Book_MAT
from models.Location import Location
from datetime import datetime

#도서픽업 작업 생성 
def get_shelf_loc(db:Session, loc_id):
    reslut = db.query(Location).filter(Location.LOC_ID == loc_id).first()
    if not reslut:
        return None
    return reslut


# 책 정보 조회
def get_books(isbn: str,db:Session):
    return db.query(BookInfo).filter(BookInfo.ISBN == isbn ).first()

def get_locID(isbn: str, db:Session):
    return db.query(Book_MAT).filter(Book_MAT.ISBN == isbn).first()

def get_loc_info(locid: int, db: Session):
    return db.query(Location).filter(Location.LOC_ID == locid).first()


#도서 정보 업데이트
def get_barcode(barcode: str, db:Session):
    return db.query(Book_MAT).filter(Book_MAT.Barcode == barcode).first()

def update_books_info(barcode: str, lastscan: datetime ,db:Session):
    bookmat = db.query(Book_MAT).filter(Book_MAT.Barcode == barcode).first()
    
    if not bookmat:
        return None
    bookmat.LastScanTime = lastscan
    db.commit()
    db.refresh(bookmat)

    return bookmat

#보관함 정보 업데이트 (새로운 픽업 데이터 넣기)
