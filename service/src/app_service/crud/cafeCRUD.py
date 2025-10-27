from sqlalchemy.orm import Session
from schemas.cafe import Order
from models.BeverageOrder import BeverageOrder
from models.Beverage import Beverage
#주문 생성

def create_order(db: Session, order: BeverageOrder):
    db.add(order)
    db.commit()
    db.refresh(order)
    return order

def get_beverage(db: Session):
    return db.query(Beverage).first()

def get_beverageOrder(db: Session):
    return db.query(BeverageOrder).first()

def get_menu(db: Session):
    return db.query(Beverage).all()

def update_order_status(orderID: int,status: str,db:Session):
    _status = db.query(BeverageOrder).filter(BeverageOrder.OrderID == orderID).first()
    _status.OrderStatus = status
    db.commit()
    db.refresh(_status)
    return _status