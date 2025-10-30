from sqlalchemy.orm import Session
from sqlalchemy import or_
from models.Member import Member
from models.BookInfo import BookInfo
from models.Book_MAT import Book_MAT
from models.Location import Location
from models.Rental import Rental

#회원 인증 
def get_member_by_cardnumber(db: Session, cardnumber: str):
    return db.query(Member).filter(Member.CardNumber == cardnumber).first()


#도서 조회
def get_books(db: Session, keyword: str = None, page: int = 1, per_page: int = 5):
    query = db.query(BookInfo)

    if keyword:
        search = f"%{keyword}%"
        query = query.filter(
            or_(
                BookInfo.Title.like(search),
                BookInfo.Author.like(search),
                BookInfo.Publisher.like(search),
            )
        )
    total = query.count()
    books = query.offset((page - 1) * per_page).limit(per_page).all()
    
    return total, books

#도서 조회 - Book_MAT 조회
def get_books_mat(db: Session, isbn: str):
    mat = db.query(Book_MAT).filter(Book_MAT.ISBN == isbn).all()
    if not mat:
        return None
    return mat

def get_books_rental(db: Session, copy_id: str):
    return_date = db.query(Rental.ReturnDate).filter(Rental.CopyID == copy_id).all()
    if not return_date:
        return None
    return return_date

#도서 조회 후 위치 보기

def get_book_location(db: Session, isbn: str):
    locID = db.query(Book_MAT).filter(Book_MAT.ISBN == isbn).first()
    if not locID:
        return None
    return db.query(Location).filter(Location.LOC_ID == locID.LOC_ID).first()


#도서 직접 및 픽업 대출

def get_copy_book(db:Session, barcode: str):
    copy = db.query(Book_MAT).filter(Book_MAT.Barcode == barcode).first()
    if not copy:
        return None
    return copy

def create_rental(db:Session, db_rantal: Rental):
    db.add(db_rantal)
    db.commit()
    db.refresh(db_rantal)
    return db_rantal

#픽업대 상태 조회
def get_box_status(db: Session):
    box_status = db.query(Location).filter(Location.LocationName.like(f"%{'도서 픽업대'}%"),
                                    Location.CurrentStatus == "사용가능")\
                                    .first()
    if not box_status:
        return None
    return box_status

#회원 정보 확인용
def get_member_info(db:Session, member_id: str):
    return db.query(Member).filter(Member.MemberID == member_id).first()

#위치 정보 업데이터
def update_location(db: Session, loc_id: int):
    loc = db.query(Location).filter(Location.LOC_ID == loc_id).first()
    if not loc:
        return None  
    
    loc.CurrentStatus = "사용불가"
    db.commit()
    db.refresh(loc)

    return loc

def get_book_info(db:Session, isbn: str):
    book_id = db.query(BookInfo).filter(BookInfo.ISBN == isbn).first()
    if not book_id:
        return None
    return book_id