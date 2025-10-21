from PyQt5 import uic
from PyQt5.QtWidgets import QApplication, QMainWindow, QStackedWidget, QMessageBox
import sys, os
import requests


class OrderManager:
    def __init__(self):
        self.next_number = 1
        self.in_progress = []
        self.completed = []
        self.next_pickup_slot = 1
        # 메뉴 데이터 (필요 시 외부/DB로 대체)
        self.menu = {
            "americano": {"name": "아메리카노", "price": 2000},
            "latte": {"name": "카페라떼", "price": 2500},
            "bboong": {"name": "붕붕드링크", "price": 1500},
        }

class Kiosk(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("카페 키오스크")
        self.resize(540, 720)

        self.order_mgr = OrderManager()
        self.selected_item = None  # ✅ 현재 선택 메뉴 상태

        self.stack = QStackedWidget()
        self.setCentralWidget(self.stack)
        self.pages = {}  # {"start": QWidget, "menu": QWidget, "menu_detail": QWidget, "payment": QWidget}

        self.show_page("start")

    def get_page(self, name: str):
        if name in self.pages:
            return self.pages[name]

        ui_file = f"{name}.ui"
        if not os.path.exists(ui_file):
            QMessageBox.information(self, "안내", f"{ui_file} 파일을 찾을 수 없습니다.")
            return None

        page = uic.loadUi(ui_file)
        self.pages[name] = page
        self.stack.addWidget(page)

        # ✅ 페이지별 1회성 이벤트 연결
        if name == "start":
            # 예: 시작 → 메뉴
            if hasattr(page, "btnGoToMenu"):
                # page.btnGoToMenu.clicked.connect(lambda: self.show_page("menu"))
                page.btnGoToMenu.clicked.connect(lambda: self.send_request(page))
                # labelStatus
                
                
            # 다른 버튼이름이라면 여기서 맞춰 연결 (예: btnStart)

        elif name == "menu":
            # 각 메뉴 버튼을 개별 연결
            if hasattr(page, "btnAmericano"):
                page.btnAmericano.clicked.connect(lambda _, k="americano": self.select_item_and_go(k))
            if hasattr(page, "btnLatte"):
                page.btnLatte.clicked.connect(lambda _, k="latte": self.select_item_and_go(k))
            if hasattr(page, "btnBboongDrink"):
                page.btnBboongDrink.clicked.connect(lambda _, k="bboong": self.select_item_and_go(k))

            # 뒤로가기 → start (버튼이 있을 경우)
            if hasattr(page, "btnGoToStart"):
                page.btnGoToStart.clicked.connect(lambda: self.show_page("start"))

        elif name == "menu_detail":
            # 주문 버튼(상세 → 결제)
            if hasattr(page, "btnOrder"):
                page.btnOrder.clicked.connect(lambda: self.show_page("payment"))
            # 취소/뒤로가기 → menu
            if hasattr(page, "btnCancel"):
                page.btnCancel.clicked.connect(lambda: self.show_page("menu"))

        elif name == "payment":
            # 결제 취소 → menu (또는 start 원하는 곳으로)
            if hasattr(page, "btnCancel"):
                page.btnCancel.clicked.connect(lambda: self.show_page("menu"))
            # 결제 성공 처리 버튼이 있다면 연결 (예: btnPay)
            # if hasattr(page, "btnPay"):
            #     page.btnPay.clicked.connect(self.on_pay)

        return page

    def show_page(self, name: str):
        page = self.get_page(name)
        if page is not None:
            self.update_page(name, page)  # ✅ 표시 직전 데이터 주입/갱신
            self.stack.setCurrentWidget(page)

    # ✅ 메뉴 선택 + 상세 페이지로 이동
    def select_item_and_go(self, key: str):
        data = self.order_mgr.menu.get(key)
        if not data:
            QMessageBox.warning(self, "오류", "존재하지 않는 메뉴입니다.")
            return
        self.selected_item = data
        self.show_page("menu_detail")

    # ✅ 페이지별 UI 갱신 지점(데이터 바인딩)
    def update_page(self, name: str, page):
        if name == "menu_detail":
            if hasattr(page, "labelOrder") and self.selected_item:
                page.labelOrder.setText(f"{self.selected_item['name']} ({self.selected_item['price']}원)")

        elif name == "payment":
            if hasattr(page, "labelOrder") and self.selected_item:
                page.labelOrder.setText(f"{self.selected_item['name']} ({self.selected_item['price']}원)")

    # def on_pay(self):
    #     # RFID 등 결제 완료 조건 확인 후 서버 전송 로직
    #     pass

    def send_request(self, page):
        try:
            url = "http://192.168.0.132:8888/cafe"
            resp = requests.get(url, timeout=3)  # ❗ UI가 3초 멈출 수 있음
            resp.raise_for_status()
            data = resp.json()

            # ✅ FIX 2: self.page가 아니라 넘긴 page 사용
            if hasattr(page, "labelStatus"):
                page.labelStatus.setVisible(True)
                page.labelStatus.setText(f"서버 응답: {data}")
        except Exception as e:
            if hasattr(page, "labelStatus"):
                page.labelStatus.setVisible(True)
                page.labelStatus.setText(f"에러 발생: {e}")

            
if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = Kiosk()
    w.show()
    sys.exit(app.exec_())
