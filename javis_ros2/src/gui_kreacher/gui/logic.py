import sys
import json
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtNetwork import QNetworkAccessManager, QNetworkRequest, QNetworkReply
from PyQt5.QtCore import QUrl, QByteArray
from PyQt5 import uic

class CafeApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.drink_type = None
        
        # 서버 설정
        self.server_url = 'http://192.168.0.132:8080/cafe/order'  # 전체 URL
        
        # QNetworkAccessManager 생성
        self.network_manager = QNetworkAccessManager()
        self.network_manager.finished.connect(self.on_request_finished)
        
        uic.loadUi('main0.ui', self)
        self.pushButton.clicked.connect(self.show_order_page)
        
    def show_order_page(self):
        uic.loadUi('order.ui', self)
        # 붕붕드링크 버튼은 비활성화 상태이므로 연결하지 않음
        self.americanoBtn.clicked.connect(self.order_americano)  # 아메리카노 버튼
        self.homeButton.clicked.connect(self.go_home)  # 홈으로 버튼
        
    def order_americano(self):
        self.drink_type = None
        uic.loadUi('order2.ui', self)
        self.iceButton.clicked.connect(self.select_ice)  # ICE 버튼
        self.hotButton.clicked.connect(self.select_hot)  # HOT 버튼
        self.homeButton.clicked.connect(self.go_home)  # 홈으로 버튼
    
    def select_ice(self):
        self.drink_type = '아이스'
        self.statusbar.showMessage("ICE 아메리카노가 선택되었습니다", 1000)
        from PyQt5.QtCore import QTimer
        QTimer.singleShot(1000, self.go_to_order3)
    
    def select_hot(self):
        self.drink_type = '핫'
        self.statusbar.showMessage("HOT 아메리카노가 선택되었습니다", 1000)
        from PyQt5.QtCore import QTimer
        QTimer.singleShot(1000, self.go_to_order3)

    def go_to_order3(self):
        uic.loadUi('order3.ui', self)
        self.homeButton.clicked.connect(self.go_home)
        self.payButton.clicked.connect(self.go_to_order4)
    
    def go_to_order4(self):
        self.statusbar.showMessage("주문 처리중...", 2000)
        
        # 주문 정보 생성
        beverage_name = f"{self.drink_type}아메리카노" if self.drink_type else "아메리카노"
        
        order_data = {
            "totalAmount": 1500.0,
            "paymentInfo": "카드",
            "orderStatus": "접수",
            "orderDetail": [
                {
                    "beverageName": beverage_name,
                    "quantity": 1
                }
            ]
        }
        
        # QNetworkAccessManager로 주문 전송
        self.send_order(order_data)
        
        # order4.ui로 이동
        uic.loadUi('order4.ui', self)
    
    def send_order(self, order_data):
        """QNetworkAccessManager를 사용해 POST 요청 전송"""
        try:
            # URL 설정
            url = QUrl(self.server_url)
            request = QNetworkRequest(url)
            
            # 헤더 설정
            request.setHeader(QNetworkRequest.ContentTypeHeader, "application/json; charset=utf-8")
            
            # JSON 데이터 준비
            json_data = json.dumps(order_data, ensure_ascii=False)
            data = QByteArray(json_data.encode('utf-8'))
            
            # POST 요청 전송 (비동기)
            self.network_manager.post(request, data)
            
            print(f"주문 전송: {json_data}")
            
        except Exception as e:
            print(f"요청 생성 오류: {e}")
            self.statusbar.showMessage(f"오류 발생: {e}", 3000)
    
    def on_request_finished(self, reply):
        """네트워크 요청 완료시 호출되는 콜백"""
        error = reply.error()
        
        if error == QNetworkReply.NoError:
            # 성공
            response_data = reply.readAll()
            response_text = bytes(response_data).decode('utf-8')
            
            try:
                response_json = json.loads(response_text)
                order_id = response_json.get('OrderID', 'N/A')
                order_status = response_json.get('OrderStatus', 'N/A')
                
                self.statusbar.showMessage(
                    f"주문 완료! OrderID: {order_id}, Status: {order_status}", 
                    3000
                )
                print(f"서버 응답: {response_json}")
                
            except json.JSONDecodeError:
                print(f"JSON 파싱 오류: {response_text}")
                self.statusbar.showMessage("응답 파싱 오류", 3000)
        else:
            # 오류 처리
            error_string = reply.errorString()
            print(f"네트워크 오류: {error_string}")
            self.statusbar.showMessage(f"주문 전송 실패: {error_string}", 3000)
        
        reply.deleteLater()

    def go_home(self):
        # 모든 화면에서 홈으로 돌아가기 기능
        uic.loadUi('main0.ui', self)
        self.pushButton.clicked.connect(self.show_order_page)
        # 상태 초기화
        self.drink_type = None
        self.statusbar.clearMessage()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = CafeApp()
    window.show()
    sys.exit(app.exec_())