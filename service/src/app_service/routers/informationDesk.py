from fastapi import APIRouter, HTTPException, status, Depends, Query, responses as rs
from database import SessionLocal
from sqlalchemy.orm import Session
from schemas.informationDesk import *
from schemas import robot_control_system as rcs
from crud import infodeskCRUD
from datetime import date, timedelta
from models.Rental import Rental
from routers import robot_control_system as rcsr
from crud import robot_control_system_CURD as rcsc
router = APIRouter(prefix="/infodesk", tags=["Infodesk"])


def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()


#회원 인증
@router.post("/auth", response_model=AuthenticationResult)
def authenticate(auth: Authentication, db: Session = Depends(get_db)):
    member = infodeskCRUD.get_member_by_cardnumber(db, auth.cardNumber)
    if not member:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="유효하지 않은 회원 카드입니다."
        )
    
    return AuthenticationResult(
        memberID=str(member.MemberID),
        name=member.Name,
        memberType=member.MemberType.value
    )

#도서 조회
@router.get("/books", response_model=BookSearch)
def get_bookInfo(
    keyword: str = Query(None, description="검색 키워드"),
    page: int = Query(1, ge=1, description="페이지 번호"),
    per_page: int = Query(5, ge=1, le=30, description="페이지당 데이터 수"),
    db: Session = Depends(get_db)
):
    total, books = infodeskCRUD.get_books(db, keyword, page, per_page)

    #Book_MAT에 책이 있어야 대출가능(1)
    #Rental에 반납이 되어야 대출가능(2)
    results = []
    for book in books:
        books_count = 0 #같은 책이 여러 권 이므로 대여가능 책 구별을 위한 카운터
        book_states = infodeskCRUD.get_books_mat(db, book.ISBN)
        if not book_states: #도서관에 없는 책이면 대여불가로 바로 리스에 추가
            results.append(
                 BookInfo(
            isbn=book.ISBN,
            title=book.Title,
            author=book.Author,
            publisher=book.Publisher,
            status=LoanStatus.UNAVAILABLE)
            )
            continue

        for book_state in book_states: #도서관에 있는 도서들
            book_rental = infodeskCRUD.get_books_rental(db, book_state.CopyID)
            if not book_rental: #도서관에 도서는 있지만 대출중인 도서
                continue
            else:
                books_count += 1
        if books_count > 0:
            results.append(
                    BookInfo(
                isbn=book.ISBN,
                title=book.Title,
                author=book.Author,
                publisher=book.Publisher,
                status=LoanStatus.AVAILABLE)
            )
        else:
            results.append(
                    BookInfo(
                isbn=book.ISBN,
                title=book.Title,
                author=book.Author,
                publisher=book.Publisher,
                status=LoanStatus.AVAILABLE)
            )

    return BookSearch(
        total=total,
        page=page,
        perPage=per_page,
        result=results
    )

#도서 조회 후 위치 보기
@router.get("/books/{isbn}", response_model=BookLocation)
def get_book_location(isbn: str, db:Session = Depends(get_db)):
    #db에서 isbn을 조회후  위치 정보를 전송
    loc = infodeskCRUD.get_book_location(db, isbn)
    if  not loc:
        return BookLocation(
            locationName="",
            locType="",
            zoneName="",
            floor="",
            message="해당 도서는 현재 없습니다. 안내 데스크에 문의바랍니다."
        )
    return BookLocation(
        locationName=loc.LocationName,
        locType=loc.LOC_TY,
        zoneName=loc.ZoneName,
        floor=loc.Floor
    )

# 도서 직접 대출
@router.post("/books/direct", response_model=Loan)
def create_directLoan(
    memberID : str,
    _barcode : str,
    db : Session = Depends(get_db)):
    bookMat = infodeskCRUD.get_copy_book(db, _barcode)

    if not bookMat.Barcode:
        raise HTTPException(status_code=404, detail="해당 바코드의 도서를 찾을 수 없습니다.")
    
    copyid = bookMat.CopyID
    memberid = int(memberID)
    loan_date = date.today()
    due_date = loan_date + timedelta(days=14)
    return_date = None
    extension_count = 0
    late_fee = 0

    rentalDate = Rental(
        CopyID = copyid,
        MemberID = memberid,
        LoanDate = loan_date,
        DueDate = due_date,
        ReturnDate = return_date,
        ExtensionCount = extension_count,
        LateFee = late_fee
    )

    infodeskCRUD.create_rental(db, rentalDate)
   
    return Loan(
        loanDate=loan_date,
        dueDate=due_date
    )



# 도서 픽업 대출
@router.post("/books/pickup", response_model=Loan)
def create_pickupLoan(
    memberID : str,
    barcode : str,
    db : Session = Depends(get_db)):
    try:

        member = infodeskCRUD.get_member_info(db, memberID)

        if not member.MemberID:
            raise HTTPException(status_code=400, detail="해당 회원은 존재하지 않습니다.")

        bookMat = infodeskCRUD.get_copy_book(db, barcode) #예약할 도서 재고

        if not bookMat.Barcode:
            raise HTTPException(status_code=404, detail="해당 바코드의 도서를 찾을 수 없습니다.")
        
        locations = infodeskCRUD.get_box_status(db) #픽업대 위치

        if not locations:
            return Loan(
            loanDate=None,
            dueDate=None,
            locationName="자리없음")
    
        #Rental 정보 생성
        copyid = bookMat.CopyID
        memberid = int(memberID)
        loan_date = date.today()
        due_date = loan_date + timedelta(days=14)
        return_date = None
        extension_count = 0
        late_fee = 0

        rentalDate = Rental(
            CopyID = copyid,
            MemberID = memberid,
            LoanDate = loan_date,
            DueDate = due_date,
            ReturnDate = return_date,
            ExtensionCount = extension_count,
            LateFee = late_fee
        )

        infodeskCRUD.create_rental(db, rentalDate)# 예약정보 생성
        infodeskCRUD.update_location(db, locations.LOC_ID) #도서 상태 업데이트
        
        #로봇 서버에 도서 픽업 작업 생성 요청 부분
        shelf_loc = rcsc.get_shelf_loc(db, bookMat.LOC_ID)
        if not shelf_loc:
            raise HTTPException(status_code=404, detail="존재하지 않은 도서 위치입니다.")
        send_to_pickup_task = rcs.BooksPickupTask(
            taskName="pickup_book",
            book_id=barcode, 
            storage_id=locations.LOC_ID,
            book_pick_pose=rcs.BookLoc(x=locations.CoordinateX, y=locations.CoordinateY, z=0.0).model_dump(),
            storage_approach_location=rcs.StorageLoc(
                x=locations.CoordinateX,
                y=locations.CoordinateY,
                theta=locations.CoordinateZ
            ).model_dump(),
            storage_slot_pose=rcs.StoragePickLoc(x=locations.CoordinateX, y=locations.CoordinateY, z=0.0).model_dump(),
            shelf_approach_location=rcs.ShelfLoc(
                x=shelf_loc.CoordinateX,
                y=shelf_loc.CoordinateY,
                theta=shelf_loc.CoordinateZ
            ).model_dump()
        )
        
        rcsr.book_pickup_request(send_to_pickup_task)#로봇 서버에 요청
        return Loan(
            loanDate=loan_date,
            dueDate=due_date,
            locationName=locations.LocationName
            #preparationTime=, 이 부분은 아직 보류
        )
    except HTTPException as e:
        return {"error":e.detail}
    




   
    
