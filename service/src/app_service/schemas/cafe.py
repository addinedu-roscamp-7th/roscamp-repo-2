from pydantic import BaseModel
from typing import List
from enum import Enum
from models.Beverage import Beverage

#요청
# 주문 상세 (음료 ID + 수량)
class OrderDetail(BaseModel):
    beverageName: str
    quantity: int

# 주문 전체 구조
class Order(BaseModel):
    totalAmount: float
    paymentInfo: str
    orderStatus: str   # 접수, 완료, 취소
    orderDetail: List[OrderDetail]


class OrderStatus(str, Enum):
    접수 = '접수'
    제조중 = '제조중'
    완료 = '완료'
    픽업대기 = '픽업대기'


class OrderState(BaseModel):
    orderId: int
    orderStatus: OrderStatus

class OrderStateUpdate(BaseModel):
    orders: List[OrderState]


#응답
class OrderStatusEnum(str, Enum):
    접수 = "접수"
    제조중 = "제조중"
    완료 = "완료"
    픽업대기 = "픽업대기"
#주문상태
class OrderList(BaseModel):
    orderID: int
    orderStatus: OrderStatusEnum

#메뉴 조회
class BeverageSearch(BaseModel):
    beverageName: str
    price: float
    description: str
    stockStatus: str

class BeverageList(BaseModel):
    beverage: List[BeverageSearch]