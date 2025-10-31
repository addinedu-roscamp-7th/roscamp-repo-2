from fastapi import APIRouter, HTTPException, WebSocket, Depends
from fastapi.responses import JSONResponse
from schemas.cafe import Order, OrderList,BeverageList, BeverageSearch
from sqlalchemy.orm import Session
from database import SessionLocal
from crud import cafeCRUD
from models.BeverageOrder import BeverageOrder
from routers.robot_control_system import menu_preparation_request

router = APIRouter(prefix="/cafe", tags=["Cafe Orders"])

def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()


# 주문 생성
@router.post("/order", response_model=OrderList)
def create_order(order: Order, db: Session = Depends(get_db)):
   try:
        OrderState = ['접수','제조중','완료','픽업대기']
        if order.orderStatus not in OrderState:
            return JSONResponse(
            status_code=400,
            content={"error": f"허용되지 않은 OrderStatus: {order.orderStatus}"}
        )

        member_id = 6
        total_amount = order.totalAmount
        payment_info = order.paymentInfo
        order_status = order.orderStatus
        
        
        beverage = BeverageOrder(
                MemberID = member_id,
                TotalAmount = total_amount,
                PaymentInfo = payment_info,
                OrderStatus = order_status
        )
        cafeCRUD.create_order(db, beverage)
       
        beverageorder = cafeCRUD.get_beverageOrder(db)

        #메뉴 제조 요청

        menu_preparation_request(order)
        return OrderList(
            orderID= beverageorder.OrderID,
            orderStatus= beverageorder.OrderStatus
        )
   except HTTPException as e:
       return {"error":e.detail}


#메뉴 조회
@router.get("/menu", response_model=BeverageList)
def getMenu(db: Session = Depends(get_db)):
    menus = cafeCRUD.get_menu(db)
    menulist = []
    for menu in menus:
        menu_info = BeverageSearch(
            beverageName = menu.BeverageName,
            price= menu.Price,
            description= menu.Description,
            stockStatus= menu.StockStatus
        )
        menulist.append(menu_info)
    return BeverageList(
        beverage= menulist
    )

#주문 상태 업데이트
@router.websocket('/order/state')
async def websocket_order_state(websocket: WebSocket,db: Session = Depends(get_db)):
    await websocket.accept()
    print("Client connected")
    try:
        while True:
            try:
                data = await websocket.receive_json()
            except Exception as e:
                print("Invalid JSON:", e)
                continue

            orders = data.get("Orders", [])

            for order in orders:
                order_id = order.get("OrderID")
                status = order.get("OrderStatus")
                print(f'Order {order_id} -> Status{status}')
                if order_id is None or status is None:
                    print("Invalid order data:", order)
                    continue
                cafeCRUD.update_order_status(order_id, status,db)

            await websocket.send_json({"message": "Orders updated successfully"})
    
    except Exception as e:
        print("WebSocket error:", e)
        await websocket.close()